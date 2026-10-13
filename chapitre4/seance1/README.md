# Un même robot : cinématique et dynamique

Cette simulation nominale accompagne le chapitre 4 de GEN6323. Elle compare le **même robot différentiel**, le **même chemin lisse en huit** et la **même boucle de correction de pose**, à deux cadences : un tour en 32 s ou en 8 s. Les paramètres sont illustratifs; ils ne proviennent pas de l'identification d'un robot commercial.

Ouvrir `chapitre4_simulation.html` dans un navigateur pour la démonstration autonome. Le fichier contient ses données : aucun serveur, réseau ou module externe n'est nécessaire. Choisir la durée, lancer la lecture ou déplacer le curseur. La vitesse de lecture modifie seulement l'animation, pas la simulation. Les silhouettes sont agrandies pour la lecture.

## Repères, états et conventions

Le repère inertiel est plan. La pose du milieu de l'essieu est `(x,y,θ)`. L'axe `x_B` du robot pointe vers l'avant, `y_B` vers sa gauche; `θ` croît dans le sens antihoraire. La vitesse longitudinale réelle est `v`; la vitesse angulaire réelle est `ω = θ̇`. Les roues gauche et droite ont le même rayon `r`; `b` est la voie complète. Les couples `τ_L` et `τ_R` sont positifs quand ils entraînent le robot vers l'avant. Ainsi `τ_R > τ_L` produit un moment de lacet positif.

Les indices et symboles restent distincts :

- `(x_d,y_d,θ_d,v_d,ω_d)` : mouvement de référence;
- `(v_c,ω_c)` : consignes calculées par la boucle externe, comprenant la correction de pose;
- `(v,ω)` : vitesses réellement réalisées;
- `(τ_L*,τ_R*)` : couples demandés par la boucle interne;
- `(τ_L,τ_R)` : couples appliqués après une éventuelle saturation.

## Une référence régulière, sans démarrage artificiel

Avec `s=2πt/T`, on pose `x_d=A sin s`, `y_d=B sin(2s)`, avec `A=3 m` et `B=1,6 m`. La courbe a une tangente non nulle partout : si `cos s=0`, alors `cos(2s)=-1`. On définit donc sans singularité

```
θ_d = atan2(ẏ_d,ẋ_d)  (orientation modulo 2π)
v_d = sqrt(ẋ_d²+ẏ_d²) > 0
ω_d = (ẋ_d ÿ_d−ẏ_d ẍ_d)/(ẋ_d²+ẏ_d²).
```

Le programme calcule analytiquement `v̇_d` et `ω̇_d`; il ne différencie pas des échantillons bruités. Tous les modèles commencent à la pose de référence, avec ses vitesses initiales. Cette condition isole le suivi d'une manœuvre; elle ne représente pas un départ arrêté. Le mouvement pourrait être précédé d'une phase de mise en vitesse, non représentée.

Le passage de 32 s à 8 s multiplie les vitesses de référence par 4 et leurs dérivées temporelles par 16. La géométrie ne change pas.

## Boucle externe commune

La position désirée moins la position réelle est exprimée dans le repère du robot :

```
[e_x,e_y]^T = R(θ)^T ([x_d,y_d]^T−[x,y]^T)
e_θ = atan2(sin(θ_d−θ),cos(θ_d−θ))
v_c = v_d cos(e_θ)+k_x e_x
ω_c = ω_d+k_y v_d e_y+k_θ sin(e_θ).
```

Gains constants identiques dans les expériences : `k_x=1,5 s⁻¹`, `k_y=1,2 m⁻²`, `k_θ=2 s⁻¹`. La dimension de `k_y` est importante : `k_y v_d e_y` est une vitesse angulaire. On ne revendique aucune garantie globale pour cette loi avec le robot dynamique saturé. Les consignes de vitesse ne sont pas bornées artificiellement; les limites sont appliquées aux couples physiques, ce qui rend les conséquences de la saturation visibles.

## Trois réalisations de ces mêmes consignes

Dans tous les cas, `ẋ=v cos θ`, `ẏ=v sin θ`, `θ̇=ω` restent les relations cinématiques.

1. **Cinématique idéale** : le modèle suppose directement `v=v_c`, `ω=ω_c`. Il ne calcule pas d'efforts. Avec l'initialisation choisie, il suit exactement la référence à l'erreur numérique près.
2. **Dynamique, couples non bornés** : le robot a une inertie et une boucle de vitesse finie; on ignore uniquement la limite des couples pour isoler l'effet de la bande passante.
3. **Dynamique, couples bornés** : même robot, même boucle interne, mais chaque couple est borné à `±1,4 N·m`.

La dynamique est

```
m_e v̇ = F_a−c_v v,        F_a = (τ_L+τ_R)/r
J_e ω̇ = M_a−c_ω ω,        M_a = b(τ_R−τ_L)/(2r).
```

La boucle interne compense nominalement la dissipation, sans anticiper l'accélération :

```
F_a* = c_v v + m_e (v_c−v)/T_v
M_a* = c_ω ω + J_e (ω_c−ω)/T_ω
τ_L* = r F_a*/2−r M_a*/b
τ_R* = r F_a*/2+r M_a*/b.
```

Sans saturation, substitution dans le modèle donne exactement `T_v v̇=v_c−v` et `T_ω ω̇=ω_c−ω`. Ces constantes sont celles d'une **boucle de vitesse fermée**, pas une affirmation sur le moteur nu. Avec saturation individuelle des roues, cette égalité n'est plus vraie; translation et rotation partagent les couples disponibles.

| Paramètre | Valeur | Signification |
|---|---:|---|
| `m_e` | 12 kg | Inertie effective en translation |
| `J_e` | 0,70 kg·m² | Inertie effective en lacet |
| `r` | 0,10 m | Rayon d'une roue |
| `b` | 0,45 m | Voie complète |
| `c_v` | 1 N·s/m | Dissipation longitudinale |
| `c_ω` | 0,08 N·m·s | Dissipation en lacet |
| `T_v` | 0,25 s | Constante de temps de vitesse longitudinale |
| `T_ω` | 0,18 s | Constante de temps de vitesse angulaire |
| `τ_max` | 1,40 N·m | Limite symétrique par roue |

Les inerties effectives incluent déjà l'éventuelle inertie de rotation des roues et transmissions ramenée aux coordonnées `(v,ω)`. On ne l'ajoute pas une seconde fois.

## Résultats et portée de l'exemple

L'erreur de position compare les positions **au même instant**, `||p(t)−p_d(t)||`. Elle inclut donc le retard sur la trajectoire. Ce n'est pas la distance minimale à la courbe géométrique. La RMS est `sqrt((1/T) intégrale_0^T ||p−p_d||² dt)`.

| Durée | Modèle | RMS de position | Maximum | Au moins un couple saturé |
|---:|---|---:|---:|---:|
| 32 s | Cinématique idéale | < 10⁻¹⁰ m | < 10⁻¹⁰ m | Sans objet |
| 32 s | Dynamique sans borne | 0,02465 m | 0,04281 m | Sans objet |
| 32 s | Dynamique avec borne | 0,02465 m | 0,04281 m | 0 % |
| 8 s | Cinématique idéale | < 10⁻¹⁰ m | < 10⁻¹⁰ m | Sans objet |
| 8 s | Dynamique sans borne | 0,30691 m | 0,54443 m | Sans objet |
| 8 s | Dynamique avec borne | 1,54764 m | 3,11705 m | 87,09 % |

Les courbes dynamiques lentes se superposent parce que la limite n'est jamais active. Une erreur de 2,5 cm peut être acceptable ou non selon l'application : « cinématique suffisante » dépend aussi de la précision demandée.

Pour un suivi **exact**, les efforts nominaux nécessaires seraient

```
F_a,d = m_e v̇_d+c_v v_d,   M_a,d = J_e ω̇_d+c_ω ω_d
τ_L,d = r F_a,d/2−r M_a,d/b, τ_R,d = r F_a,d/2+r M_a,d/b.
```

Le pic requis vaut 0,15287 N·m à 32 s et 2,18684 N·m à 8 s. Le cas rapide dépasse donc la limite de 1,40 N·m **même avant de choisir le régulateur**. La dynamique peut aider à anticiper les efforts, augmenter la bande passante ou choisir une cadence réalisable; elle ne crée pas des couples supplémentaires. Le très grand écart du cas saturé provient également de la correction de pose qui sollicite encore les actionneurs lorsqu'elle reçoit du retard. Ce résultat n'est pas une limite fondamentale sur le suivi géométrique à une autre cadence.

Le suivi rapide présente une accélération latérale de référence maximale d'environ 4,13 m/s² (contre 0,258 m/s² pour le lent). L'adhérence nécessaire est supposée disponible. La simulation ne démontre pas que ces mouvements sont réalisables par un robot réel donné.

## Hypothèses et exclusions

Sol plan; centre de masse au milieu de l'essieu; roulement sans glissement; adhérence suffisante; modèle et état connus exactement; couples appliqués instantanément après saturation. Pas de dynamique électrique, courbe couple-vitesse, limite de batterie, bruit, retard de calcul, glissement ni perturbation. Les couples non bornés sont un **cas de comparaison mathématique**, pas un actionneur réalisable. Les résultats illustrent un mécanisme et ne remplacent pas une validation expérimentale.

## Reproduction et vérifications

Installer les dépendances indiquées dans le README à la racine du dépôt (Python, NumPy ≥ 2.0 et Matplotlib). Depuis cette racine :

```sh
python chapitre4/seance1/simulation_chapitre4.py
```

Depuis le dossier `chapitre4/seance1/`, la commande courte est `python simulation_chapitre4.py`. Le script peut aussi être appelé par son chemin absolu depuis n'importe quel autre dossier.

Par défaut, les six CSV, les JSON et le HTML régénéré sont écrits à côté du script, et les figures dans `chapitre4/seance1/figures/`. Pour choisir un autre dossier de sortie et affiner le pas :

```sh
python chapitre4/seance1/simulation_chapitre4.py --output resultats/seance1 --dt 0.001
```

Un chemin fourni à `--output` est relatif au dossier depuis lequel la commande est lancée, sauf s'il est absolu. Le fichier `chapitre4_animation_template.html` doit rester à côté du script pour régénérer l'animation.

Le pas RK4 vaut 2 ms par défaut. L'interface accepte uniquement `--dt 0.002` ou `--dt 0.001`, les deux valeurs vérifiées; toute autre valeur est rejetée avant de créer les sorties. Les consignes et saturations sont recalculées aux quatre sous-étapes du RK4. Le script recalcule automatiquement le cas rapide saturé à un pas divisé par deux (1 ms ou 0,5 ms) pour vérifier la convergence.

Vérifications enregistrées dans `chapitre4_resultats.json` :

- inversion exacte de la transformation couple/effort et identité de puissance sur 1 000 cas aléatoires;
- dérivées analytiques de la référence comparées à des différences centrées en 1 000 instants;
- non-annulation de la tangente de référence;
- suivi idéal exact et identité analytique de la boucle interne non saturée;
- respect strict des limites de couple;
- pas 2 ms → 1 ms : écart maximal de position 0,06563 mm, variation de RMS 0,00355 mm.

Les six CSV sont régénérés par le programme; ils ne sont pas stockés dans le dépôt. Le HTML livré contient déjà toutes les données nécessaires à l'animation. Il peut être ouvert sans Python et sans connexion.

Les cinq figures PDF vectorielles `sim_trajectoires`, `sim_vitesses`, `sim_couples`, `sim_erreurs` et `sim_faisabilite` sont produites dans `figures/`, à l'intérieur du dossier de sortie; les PNG correspondants permettent l'inspection. Le CSV de chaque expérience contient les états, références, consignes, couples et erreur de position. Dans les CSV du modèle idéal, les colonnes de couples sont sans objet et marquées `nan`. Le JSON de l'animation conserve des zéros internes inutilisés pour ces mêmes couples idéaux; ils ne représentent pas des couples physiques nuls.

Le vérificateur JavaScript utilise uniquement Node.js. Depuis la racine :

```sh
node chapitre4/seance1/verifier_animation_chapitre4.js
```

Il vérifie le HTML voisin du vérificateur et écrit `chapitre4_animation_verification.json` au même endroit. Il fonctionne indépendamment du dossier courant; aucun module npm n'est nécessaire.
