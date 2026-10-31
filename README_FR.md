# APM-Lua

Ensemble de scripts Lua pour ArduPilot / ArduCopter, destinés à étendre le comportement de vol à l'exécution du firmware.

> ## ⚠️ Avis d'applicabilité
> **Tous les scripts Lua n'ont été validés sur matériel réel qu'avec la version de firmware correspondante ; ils peuvent ne pas fonctionner avec toute autre version de firmware.**

## Objet du dépôt

Ce dépôt héberge des scripts Lua écrits pour le firmware APM (ArduPilot / ArduCopter). Les scripts s'exécutent sur le contrôleur de vol au moyen du moteur de scripts Lua intégré à ArduPilot et étendent ou restreignent le comportement de vol sans modifier le code source du firmware — par exemple en limitant dynamiquement la vitesse de descente manuelle en fonction de l'altitude.

Le dépôt ne contient que des scripts pouvant être copiés directement dans le répertoire `APM/scripts/` de la carte SD du contrôleur de vol. Il n'inclut ni code source de firmware, ni configuration de station au sol, ni dépendance tierce.

## Versions de firmware ciblées

- Firmware pris en charge : ArduCopter 4.6.3 et 4.7.0 (multirotor).
- Adaptation des paramètres : la version 4.6.x utilise `PILOT_SPEED_DN` / `PILOT_SPEED_UP` (unité : cm/s) ; la version 4.7.x utilise `PILOT_SPD_DN` / `PILOT_SPD_UP` (unité : m/s). Un adaptateur intégré identifie automatiquement la paire et exige la présence d'une seule des deux paires ; dans le cas contraire, le script se limite à la surveillance sans jamais écrire.
- Autres versions : aucune gestion de compatibilité n'est implémentée. Toute modification des noms ou de la sémantique des paramètres peut rendre les scripts inopérants ou provoquer un comportement inattendu.

## Arborescence

```text
.
├── LICENSE
├── README_CN.md / README_EN.md / README_FR.md / README_RU.md / README_ES.md / README_AR.md
└── scripts
    └── 近地面减速_near_ground_slowdown
        ├── near_ground_slowdown.lua
        ├── near_ground_slowdown_stage2_0p2.lua
        └── README.md
```

## Inventaire des scripts

| Script | Firmware | Description |
| --- | --- | --- |
| `near_ground_slowdown.lua` | ArduCopter 4.6.3 / 4.7.0 | En modes AltHold / Loiter, limite la vitesse de descente manuelle selon la hauteur au-dessus du point home : 0,50 m/s entre 2 m et 5 m, 0,10 m/s à 2 m ou moins |
| `near_ground_slowdown_stage2_0p2.lua` | ArduCopter 4.6.3 / 4.7.0 | Identique ci-dessus, mais le palier ≤ 2 m est porté à 0,20 m/s |

La définition des paramètres, les codes d'état et les champs de journal de chaque script sont documentés dans le `README.md` du répertoire fonctionnel correspondant.

## Notes d'utilisation

- Copier le script dans le répertoire `APM/scripts/` de la carte SD du contrôleur de vol et vérifier que `SCR_ENABLE = 1`.
- Ne charger qu'un seul script par fonction ; les deux variantes `near_ground_slowdown` partagent la table de paramètres `NGC_` et ne doivent pas être chargées simultanément.
- Les scripts n'écrivent que les paramètres de vitesse de descente et n'envoient aucune commande d'armement / désarmement / moteur / mode ; l'atterrissage repose sur le désarmement automatique natif `DISARM_DELAY`.
- La hauteur provient de la position EKF relative au point home, et non de la hauteur réelle au-dessus du sol ; le relief et la dérive barométrique influencent la décision.
- Lors de la première utilisation, retirer les hélices et vérifier au sol l'enregistrement et la restauration des paramètres avant tout vol.

## Licence

Ce projet est distribué sous GNU General Public License v3.0. Voir `LICENSE`.

## Langues

- 中文：[README_CN.md](README_CN.md)
- English：[README_EN.md](README_EN.md)
- Français：[README_FR.md](README_FR.md)
- Русский：[README_RU.md](README_RU.md)
- Español：[README_ES.md](README_ES.md)
- العربية：[README_AR.md](README_AR.md)
