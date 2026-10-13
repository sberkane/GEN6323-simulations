# Simulations — Chapitre 4, séance 2

Quatre expériences reproductibles accompagnent la commande de position et les boucles de vitesse. Les paramètres sont pédagogiques, sans identification sur un robot commercial. Le roulement sans glissement, une adhérence suffisante et des mesures parfaites sont supposés.

Installer les dépendances indiquées dans le README à la racine du dépôt (Python, NumPy et Matplotlib). Depuis cette racine :

```sh
python chapitre4/seance2/simulation_seance2.py
```

Depuis le dossier `chapitre4/seance2/`, la commande courte est `python simulation_seance2.py`. Le script peut aussi être appelé par son chemin absolu depuis n'importe quel autre dossier.

Le programme produit `resultats_seance2.json` à côté du script et quatre figures vectorielles PDF, doublées de PNG, dans `chapitre4/seance2/figures/`. Pour choisir un autre dossier de sortie et affiner le pas :

```sh
python chapitre4/seance2/simulation_seance2.py --output resultats/seance2 --dt 0.001
```

L'interface accepte uniquement `--dt 0.002` (défaut) ou `--dt 0.001`, les deux valeurs vérifiées. Ces pas divisent exactement les horizons et la date du changement de charge. Toute autre valeur est rejetée avant de créer les sorties. Chaque exécution recommence ensuite les calculs à un pas divisé par deux pour vérifier leur convergence.

Les options `--figures DOSSIER` et `--results FICHIER` permettent de rediriger séparément les figures et le JSON; elles sont prioritaires sur `--output`. Les chemins fournis sont relatifs au dossier depuis lequel la commande est lancée, sauf s'ils sont absolus. Sur un système dont la détection des polices est bloquée, Matplotlib récent accepte `MPL_IGNORE_SYSTEM_FONTS=1` et utilise ses polices incluses.

| Figure | Expérience et résultat |
| --- | --- |
| `seance2_regulation_position` | But fixe (2,1) m, départ (0,0) m, trois orientations initiales. Après 12 s, les distances sont 1,740 / 2,039 / 2,679 mm pour 0 / π/2 / −2 rad. La dernière orientation produit un recul initial. Les empreintes représentent le châssis initial et final du cas bleu. |
| `seance2_comparaison_pid` | Consigne de 1 m/s à t=0, charge résistante de −3 N à t=5 s, aucune saturation. Le P conserve une erreur de 0,0625 m/s; PI et PID éliminent cette erreur constante. Avec ces gains fixés, le dépassement initial vaut 13,53 % pour PI et 16,22 % pour PID : la dérivée n'améliore pas systématiquement la réponse. |
| `seance2_anti_emballement` | Même consigne, sans charge externe, deux roues limitées à ±0,40 N m, soit ±8 N de force totale. L'intégrale sans correction produit 50,36 % de dépassement. Le rétro-calcul donne ici 0 % et réduit le temps d'établissement à ±2 % de 5,114 à 2,620 s. Traits pleins : couple appliqué; pointillés : couple demandé. |
| `seance2_cascade_position` | Même but que la première expérience, départ au repos. La boucle de position commande deux PI avec rétro-calcul; les couples des roues sont limités séparément à ±1,40 N m. La distance finale vaut 3,845 mm; une roue au moins sature pendant 0,356 s. La simulation illustre ce cas sur 12 s, sans prouver la stabilité globale de la cascade. |

Le robot dynamique utilise m_e=12 kg, J_e=0,70 kg m², c_v=1 N s/m, c_ω=0,08 N m s, r=0,10 m et b=0,45 m (voie complète). Les couples sont équivalents aux roues; les inerties tournantes sont déjà incluses dans m_e et J_e.

La commande de position est v_c=0,6 ρ cos α et ω_c=1,5 α+0,6 sin α cos α, avec α égal à la différence angulaire principale entre direction du but et orientation réelle. La vitesse signée permet le recul. Pour le modèle cinématique idéal, α̇=−1,5 α et ρ̇=−0,6 ρ cos²α. Aucun cap final n'est imposé. La protection numérique à ρ<10⁻⁹ m n'est activée dans aucun essai; les trajectoires ne sont pas arrêtées artificiellement près du but.

Pour la vitesse longitudinale, e=v_c−v et F_a*=c_v v+m_e(k_P e+k_I I−k_D z), avec İ=e et T_f ż=v̇−z. Le P prend (k_P,k_I,k_D)=(4,0,0), le PI (4,4,0), le PID filtré utilise k_P=4, k_I=4 et k_D=0,4, avec T_f=0,05 s. Les unités sont s⁻¹, s⁻² et sans dimension. La dérivée agit sur la mesure filtrée, sans dériver l'échelon de consigne. Dans la cascade, les gains angulaires sont k_Pω=6 s⁻¹, k_Iω=9 s⁻². Le rétro-calcul ajoute (F_a−F_a*)/(m_e k_I T_aw) à İ, avec T_aw=0,30 s; son équivalent angulaire utilise J_e et les moments réellement obtenus après limitation des roues.

La méthode RK4 utilise un pas de 2 ms, puis recommence les essais à 1 ms. L'échelon de charge est traité par deux intégrations distinctes, avant et après 5 s : aucun stade RK4 n'utilise la charge future avant sa date. L'écart maximal au profil P analytique est 1,27×10⁻¹¹ m/s. Le demi-pas change au maximum la position de la cascade de 0,534 μm, sa vitesse de 6,65×10⁻⁷ m/s et sa vitesse angulaire de 2,82×10⁻⁶ rad/s. Mille essais numériques indépendants vérifient l'inversion de l'allocation et l'identité de puissance; tous les résultats et paramètres sont enregistrés dans le JSON.
