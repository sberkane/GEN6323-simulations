# Chapitre 3 — Intégrer la cinématique du robot différentiel

`integration_diff_drive_model.m` compare l'intégration d'Euler à l'intégration exacte lorsque les vitesses linéaire et angulaire restent constantes sur chaque pas. Il accompagne l'exemple de la séance 1.

## Exécuter

Ouvrir le dossier `chapitre3` comme dossier courant dans MATLAB, puis exécuter :

```matlab
integration_diff_drive_model
```

Le programme utilise uniquement les fonctions de base de MATLAB, sans boîte à outils ni données externes. Il ouvre une figure. Exécution MATLAB/Octave non vérifiée.

## Observer

Les paramètres initiaux sont v = 1 m/s et ω = 0,5 rad/s. Le robot décrit donc un cercle de rayon v/ω = 2 m. Deux calculs d'Euler emploient Δt = 0,1 s et Δt = 0,5 s. La trajectoire exacte est tracée en bleu.

Le nombre de pas reste fixé à 150 dans les deux cas : les horizons sont respectivement 15 s et 75 s. La figure illustre l'écart géométrique entre les courbes; ce n'est pas une comparaison des erreurs à un même instant final. Le deuxième calcul exact est conservé dans les variables `x_exact2`, `y_exact2`, mais n'est pas tracé séparément.

Pour une comparaison à horizon identique, adapter le nombre de pas de chaque calcul. Examiner ensuite l'écart entre les positions obtenues aux mêmes instants. Essayer aussi ω = 0 : le programme traite explicitement le mouvement rectiligne.
