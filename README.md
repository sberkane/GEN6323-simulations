# GEN6323 — Simulations de robotique mobile

**[Ouvrir les laboratoires dans le navigateur](https://sberkane.github.io/GEN6323-simulations/)** · [Voir le guide interactif](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/guide.html)

Ressources du cours GEN6323 de Soulaimane Berkane, Université du Québec en Outaouais. Observer un mouvement, modifier un réglage, puis relier les courbes aux équations du cours. L'interface et les guides sont en français.

## Commencer sans installation

Commencer par la séance 1 : les efforts sont calculés à l'avance à partir d'un mouvement souhaité, sans utiliser l'état réel du robot. Comparer le modèle nominal, les limites des actionneurs et l'erreur de modèle. Les séances suivantes introduisent la rétroaction pour corriger les écarts.

Ouvrir le site ci-dessus, choisir une expérience, puis cliquer sur **Lire**. Dans les laboratoires avec rétroaction, les missions à but fixe acceptent une cible par clic; les réglages permettent d'explorer les commandes et le bouton CSV exporte les mesures.

| Dans le cours | Expérience | Accès direct |
|---|---|---|
| Chapitre 4, séance 1 | Robot différentiel : couples prescrits, saturation et erreur d'inertie | [Boucle ouverte du robot](https://sberkane.github.io/GEN6323-simulations/chapitre4/boucle_ouverte/index.html#differentiel) |
| Chapitre 4, séance 1 | Drone plan : poussées prescrites, limites et erreur de modèle | [Boucle ouverte du drone](https://sberkane.github.io/GEN6323-simulations/chapitre4/boucle_ouverte/index.html#drone) |
| Chapitre 4, séance 2 | Rejoindre un point, cap libre | [Déplacer le robot](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar) |
| Chapitre 4, séance 2 | Rejoindre une pose, cap imposé | [Comparer point et pose](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/pose) |
| Chapitre 4, séance 2 | P, PI, PID, saturation et anti-enroulement | [Commander les moteurs](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#vitesse) |
| Chapitre 4, séance 3 | Huit avec horaire, puis cercle sans horaire | [Trajectoire](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/trajectory) · [Chemin](https://sberkane.github.io/GEN6323-simulations/chapitre4/interactif/index.html#planar/path) |
| Chapitre 4, séance 3 | Même correction de pose, trois réalisations des vitesses | [Comparer les modèles avec rétroaction](https://sberkane.github.io/GEN6323-simulations/chapitre4/seance3/chapitre4_simulation.html) |
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
python chapitre4/boucle_ouverte/differentiel.py --output resultats/differentiel --figures resultats/differentiel/figures
python chapitre4/boucle_ouverte/drone.py --output resultats/drone --figures resultats/drone/figures
python chapitre4/seance2/simulation_seance2.py --output resultats/seance2
python chapitre4/seance3/simulation_chapitre4.py --output resultats/seance3
```

Sur macOS/Linux, employer `python3` pour la création de l'environnement si `python` n'est pas disponible. Les deux pas acceptés sont `--dt 0.002` (défaut) et `--dt 0.001` (raffinement).

Les programmes de boucle ouverte produisent les données JSON/JS et les résultats numériques dans `--output`. L'option `--figures` est facultative et choisit le dossier des figures PDF/PNG; sans elle, NumPy suffit. Le différentiel produit `diff_data.json`, `diff_data.js` et `diff_resultats.json`; le drone produit `quad_data.json`, `quad_data.js` et `quad_resultats.json`. Le laboratoire livré contient déjà ses données et fonctionne sans exécuter Python.

Les scripts des séances 2 et 3 produisent leurs figures dans le sous-dossier `figures/` de `--output`. Celui de séance 3 recrée aussi une animation HTML autonome : son fichier `chapitre4_animation_template.html` doit rester près du script. Sans `--output`, chaque programme écrit près de son propre fichier; les chemins fournis explicitement sont relatifs au dossier courant.

Voir les détails : [séance 1](chapitre4/seance1/README.md), [séance 2](chapitre4/seance2/README.md), [comparaison de séance 3](chapitre4/seance3/README.md), [modèles du laboratoire avec rétroaction](chapitre4/interactif/README.md).

## Vérifier les calculs

Les scripts Python exécutent leurs contrôles numériques lors de la reproduction. Avec Node.js (version 24 vérifiée), sans `npm install` :

```sh
node chapitre4/boucle_ouverte/verifier_interface.js
node chapitre4/seance3/verifier_animation_chapitre4.js
node chapitre4/interactif/verifier_laboratoire.js
```

Ces contrôles vérifient notamment les données de l'animation, les contraintes d'effort et les comportements attendus des modèles. Ils ne constituent pas une preuve de stabilité générale. Exécution MATLAB/Octave non vérifiée.

## Organisation

```text
index.html                     Accueil à ouvrir dans le navigateur
chapitre3/                     Intégration cinématique (MATLAB)
chapitre4/boucle_ouverte/       Séance 1 : dynamique inverse et animations
chapitre4/seance1/             Navigation et ancien lien conservé
chapitre4/seance2/              Quatre études reproductibles (Python)
chapitre4/seance3/             Suivi de trajectoire avec rétroaction
chapitre4/interactif/           Laboratoire avec calculs dans le navigateur
requirements.txt               Dépendances Python vérifiées
```

Les paramètres sont pédagogiques et les mesures idéales. Les hypothèses, limites, conventions de signe et lois de commande sont précisées dans chaque guide. Ce dépôt accompagne les supports PDF, distribués séparément dans le cours. Il contient les exemples actuellement disponibles des chapitres 3 et 4.
