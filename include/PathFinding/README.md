# Grille et logique de Ben

Modifier path_settings.h puis recompiler et televerser.

```cpp
constexpr float CELL_M = 0.5f;
constexpr int FORWARD_CELLS = 2;
```

1 = 0,5 m; 2 = 1 m; 3 = 1,5 m vers le nord. Maximum autorise : ROWS-1. Le solveur reduit la distance avant une ligne interdite ou l arrivee. Les changements de colonne restent d une case. La position logique et les cases explorees sont mises a jour apres la fin du mouvement.

Les capteurs IR gauche 49 et droit 48, actifs bas, sont lus avant chaque translation, pas pendant. Un grand nombre de cases ne garantit pas l absence de panneaux inconnus plus loin. Les lignes interdites sont connues; les panneaux physiques sont detectes par les IR.

X : A=0, B=1, C=2. Y : ligne Excel 1=0, ligne 10=9. Depart (1,9) vers le nord, arrivee Y=0. FORBIDDEN combine les bits N=1 E=2 S=4 W=8. Exemple : 10 interdit est et ouest. Une declaration sur un seul cote suffit; les deux cases sont consultees. Les limites exterieures sont interdites.

Grille actuelle : A-B et B-C interdites aux lignes Excel 1,3,5,7,9; passages lateraux aux lignes 2,4,6,8,10. Adapter la matrice et le depart si ROWS ou COLUMNS change.

Ben cherche le nord, contourne droite/gauche, puis reprend le nord. Cette strategie ne garantit pas tous les labyrinthes.
