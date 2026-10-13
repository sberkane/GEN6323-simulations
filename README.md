# GEN6323 — Simulations de robotique mobile

**[Ouvrir les laboratoires dans le navigateur](https://sberkane.github.io/GEN6323-simulations/)** · [Voir le guide interactif](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/guide.html)

Ressources du cours GEN6323 de Soulaimane Berkane, Université du Québec en Outaouais. Observer un mouvement, modifier un réglage, puis relier les courbes aux équations du cours. L'interface et les guides sont en français.

## Commencer sans installation

Ouvrir le site ci-dessus. Choisir une expérience, puis cliquer sur **Lire**. Dans les missions à but fixe, un clic dans la scène choisit la cible et démarre le robot. Les curseurs permettent de comparer les réglages; le bouton CSV exporte les mesures.

| Dans le cours | Expérience | Accès direct |
|---|---|---|
| Chapitre 4, séance 1 | Même trajectoire, modèles idéal/dynamique, couples limités | [Comparer les trois modèles](https://sberkane.github.io/GEN6323-simulations/chapitre4/seance1/chapitre4_simulation.html) |
| Chapitre 4, séance 2 | Rejoindre un point, cap libre | [Déplacer le robot](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar) |
| Chapitre 4, séance 2 | Rejoindre une pose, cap imposé | [Comparer point et pose](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/pose) |
| Chapitre 4, séance 2 | P, PI, PID, saturation et anti-enroulement | [Commander les moteurs](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#vitesse) |
| Chapitre 4, séance 3 | Huit avec horaire, puis cercle sans horaire | [Trajectoire](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/trajectory) · [Chemin](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/path) |
| Chapitre 4, séance 4 | Quadrirotor 2D : gravité, attitude, position | [But fixe](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#quad) · [Trajectoire](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#quad/trajectory) |
| Chapitre 3, séance 1 | Intégration d'Euler et intégration exacte | [Programme MATLAB et instructions](chapitre3/README.md) |

**Sans connexion :** télécharger le dépôt avec **Code → Download ZIP**, décompresser entièrement, puis ouvrir `index.html`. Conserver l'arborescence. Les animations fonctionnent sans serveur, compte ni bibliothèque à installer. La lecture du code sur GitHub n'exécute pas les animations : utiliser le site ou les fichiers décompressés.

## Reproduire les figures avec Python

Python 3.12 est l'environnement vérifié. Les seules dépendances sont NumPy et Matplotlib; les versions testées sont fixées dans `requirements.txt`. Depuis la racine du dépôt :

```sh
python -m venv .venv
```

Activer l'environnement :

- macOS/Linux : `source .venv/bin/activate`
- Windows PowerShell : `.venv\Scripts\Activate.ps1`
- Windows Invite de commandes : `.venv\Scripts\activate.bat`

Puis :

```sh
python -m pip install -r requirements.txt
python chapitre4/seance1/simulation_chapitre4.py --output resultats/seance1
python chapitre4/seance2/simulation_seance2.py --output resultats/seance2
```

Sur macOS/Linux, employer `python3` pour la création de l'environnement si `python` n'est pas disponible. Les scripts produisent les figures PDF/PNG et leurs résultats numériques; celui de séance 1 recrée aussi une animation HTML autonome. Les sorties restent dans le dossier demandé. Les deux pas documentés et acceptés sont `--dt 0.002` (défaut) et `--dt 0.001` (raffinement).

Voir les détails : [séance 1](chapitre4/seance1/README.md), [séance 2](chapitre4/seance2/README.md), [modèles du laboratoire](chapitre4/interactif/README.md).

## Vérifier les calculs

Les scripts Python exécutent leurs contrôles numériques lors de la reproduction. Avec Node.js (version 24 vérifiée), sans `npm install` :

```sh
node chapitre4/seance1/verifier_animation_chapitre4.js
node chapitre4/interactif/verifier_laboratoire.js
```

Ces contrôles vérifient notamment les données de l'animation, les contraintes d'effort et les comportements attendus des modèles. Ils ne constituent pas une preuve de stabilité générale. Exécution MATLAB/Octave non vérifiée.

## Organisation

```text
index.html                     Accueil à ouvrir dans le navigateur
chapitre3/                     Intégration cinématique (MATLAB)
chapitre4/seance1/              Simulation dynamique et animation pré-calculée
chapitre4/seance2/              Quatre études reproductibles (Python)
chapitre4/interactif/           Laboratoire avec calculs dans le navigateur
requirements.txt               Dépendances Python vérifiées
```

Les paramètres sont pédagogiques et les mesures idéales. Les hypothèses, limites, conventions de signe et lois de commande sont précisées dans chaque guide. Ce dépôt accompagne les supports PDF, distribués séparément dans le cours. Il contient les exemples actuellement disponibles des chapitres 3 et 4.
