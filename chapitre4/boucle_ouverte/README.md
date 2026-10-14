# Commande en boucle ouverte

Deux expériences relient les efforts prescrits au mouvement d’un robot différentiel et d’un quadrirotor dans un plan vertical. Les commandes dépendent de la référence et du modèle nominal; elles n’utilisent aucun retour de l’état réel.

## Ouvrir les animations

Ouvrir `index.html` dans un navigateur. Les données sont incluses dans `diff_data.js` et `quad_data.js` : aucun serveur, téléchargement ou module externe n’est nécessaire. Conserver ces fichiers à côté de `app.js`, `styles.css` et `index.html`.

- `index.html#differentiel` : même huit parcouru en 32 s ou 8 s.
- `index.html#drone` : déplacement avec inclinaison et maintien nominal de l’altitude en 4 s.
- `guide.html` : équations, hypothèses et interprétation des expériences.

Choisir une situation, lancer la lecture ou déplacer le curseur. La vitesse de lecture modifie seulement l’animation. Le choix de durée modifie la manœuvre physique du robot différentiel. Les courbes montrent toute la manœuvre, avec un curseur à l’instant animé; la trace dans le plan montre le mouvement déjà parcouru.

Les trois situations sont distinctes :

| Situation | Modèle et efforts |
|---|---|
| Nominal | Modèle exact, efforts prescrits non bornés. |
| Efforts bornés | Même modèle, avec limitation individuelle des efforts appliqués. |
| Erreur de modèle | Inertie réelle de translation +10 % pour le différentiel; masse réelle +5 % pour le drone. Les efforts restent ceux du modèle nominal, sans limitation. |

Le cas nominal démarre aux états désirés et utilise des efforts qui anticipent l’accélération. Il suit donc la référence à l’erreur numérique près. L’inertie seule n’introduit pas de retard dans cette situation idéale. Une saturation agit sur l’accélération par l’intermédiaire des efforts; elle n’est pas une limitation directe de vitesse.

Le différentiel suppose un roulement sans glissement et une adhérence suffisante. Le drone conserve la gravité dans tous les cas; la traînée et la dynamique des moteurs sont négligées. Les silhouettes sont agrandies pour la lecture. Ces paramètres sont pédagogiques, sans identification d’un robot commercial.

## Reproduire les données et les figures

Les scripts utilisent Python 3, NumPy et Matplotlib. Depuis la racine du dépôt de simulations :

```sh
python chapitre4/boucle_ouverte/differentiel.py --output resultats/boucle_ouverte/differentiel --figures resultats/boucle_ouverte/differentiel/figures
python chapitre4/boucle_ouverte/drone.py --output resultats/boucle_ouverte/drone --figures resultats/boucle_ouverte/drone/figures
```

Depuis ce dossier, les commandes courtes sont `python differentiel.py` et `python drone.py`. Les scripts fonctionnent également lorsqu’ils sont appelés par leur chemin absolu depuis un autre dossier. Sans `--output`, les données sont écrites à côté du script. L’option `--figures DOSSIER` demande la production des figures PDF et PNG.

Le pas RK4 par défaut est `--dt 0.002`. Le pas de raffinement accepté est `--dt 0.001`. Les résultats comprennent les contrôles du modèle et une comparaison entre les pas de 2 ms et 1 ms. Les données affichées sont échantillonnées toutes les 20 ms.

Les sorties sont :

- `diff_data.json` / `diff_data.js` et `quad_data.json` / `quad_data.js` : référence, états, efforts, métriques et éventuel arrêt de simulation;
- `diff_resultats.json` et `quad_resultats.json` : paramètres, résultats et vérifications numériques;
- avec `--figures`, quatre figures du différentiel et trois figures du drone, chacune en PDF et PNG.

Pour afficher des données régénérées, copier les fichiers `diff_data.js` et `quad_data.js` dans le dossier de l’interface. Les fichiers JSON contiennent les mêmes données sous une forme réutilisable. Les anciens exemples de suivi avec rétroaction sont des expériences différentes; leurs figures ne sont pas les résultats de cette boucle ouverte.

## Lecture des données

Chaque expérience contient les instants `t`, la référence `reference` et les situations `cases`. Les lignes de `state` et `input` correspondent aux mêmes instants.

| Modèle | États | Efforts appliqués |
|---|---|---|
| Différentiel | `[x, y, theta, v, omega]` | `[tau_L, tau_R]` |
| Drone | `[x, z, theta, vx, vz, theta_dot]` | `[F_L, F_R]` |

`reference.input` contient les efforts prescrits. `cases[].input` contient les efforts réellement appliqués. Les angles sont en radians dans les données; l’interface affiche le tangage et les écarts angulaires en degrés. L’erreur RMS compare les positions au même instant.

Si `stop` est non nul, il indique `{t, reason}`. La lecture et la trace réelle s’arrêtent à cet instant. Un éventuel dernier état répété dans les tableaux ne représente pas un mouvement simulé après l’arrêt. Aucun rebond ni maintien en vol n’est déduit de cet état figé.

## Vérifier l’interface

Avec Node.js, sans dépendance supplémentaire :

```sh
node chapitre4/boucle_ouverte/verifier_interface.js
```

Le vérificateur contrôle les dimensions des données, les valeurs finies, les instants, l’identité des fichiers JSON et JavaScript, les routes et les commandes de lecture dans un environnement de test. Il contrôle aussi les limites d’effort et le comportement en présence d’un arrêt de simulation. La mise en page se vérifie en ouvrant l’interface dans un navigateur.

Pour étudier ensuite les corrections à partir de mesures, ouvrir le [laboratoire avec rétroaction](../interactif/index.html).
