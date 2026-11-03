# Le Rectangle - ROBUS

Projet PlatformIO pour Arduino Mega 2560, avec LibRobus incluse.
Deux programmes independants sont disponibles : ramassage du gobelet et suivi de ligne.

## Compiler et televerser

Ouvrir ce dossier dans VS Code avec PlatformIO IDE. Dans **Project Tasks**, choisir
l'environnement avant **Build** ou **Upload** :

| Environnement | Programme | Main |
|---|---|---|
| `megaatmega2560` | Scan et ramassage par bumper | `src/main.cpp` |
| `line_test` | Test independant du suivi de ligne | `src/line_test_main.cpp` |

L'environnement par defaut est `megaatmega2560`. Le suivi de ligne n'est pas encore
integre a la sequence de ramassage. Moniteur serie : **9600 bauds**, fin de ligne LF/CRLF.

```powershell
pio run -e megaatmega2560
pio run -e megaatmega2560 -t upload
pio run -e line_test
pio run -e line_test -t upload
```

## Ramassage du gobelet

La pince descend des le demarrage. Un nouvel appui sur n'importe quel bumper,
ou la commande `SCAN`, lance :

1. Ouvrir le pouce, descendre la pince et attendre 2 secondes.
2. Verifier A0, puis scanner a gauche jusqu'a la premiere detection dans la plage 10..50 cm.
3. Si l'objet est deja present ou apparait dans les 5 premiers degres, tourner de
   25 degres a droite et recommencer (trois reprises maximum).
4. Arreter les roues, afficher la distance et appliquer l'offset configure.
5. Reculer de 250 mm a cap fixe, fermer le pouce, attendre 0.5 seconde, puis relever.
6. Parcourir la meme distance en sens inverse et reprendre le cap memorise avant le scan.

Les translations de prise/retour conservent les PID de roues et de cap mais
suppriment le decalage et la correction laterale de trajectoire. Leur vitesse est
limitee a 0.12 m/s. Le retour repose sur l'odometrie, sans garantie de position
exacte en cas de glissement. Aucun capteur ne confirme que le gobelet est saisi.
Le recul de prise est fixe, independant de la distance IR affichee.

Reglages dans `src/scan_config.h`, positions servo dans `src/arm.h`.
L'offset courant est -2 degres, le scan est a 0.20 rad/s ; modifier les constantes
puis recompiler et televerser pour changer ces reglages.

Commandes serie :

- `SCAN` / `START` : sequence complete.
- `TUNE` : scan et offset seulement, sans recul, fermeture ni flip.
- `+` / `-` : pas de 1 degre dans TUNE, immediats meme sans fin de ligne.
- `OFFSET` : afficher la constante a recopier dans le code.
- `STOP` : annuler et couper les roues ; les servos conservent leur consigne.
- `STATUS`, `LIVE`, `PAUSE`, `PING` : diagnostic.

Les bumpers sont filtres a 40 ms. Un bouton maintenu ne relance pas la sequence.
Les appuis pendant une sequence sont ignores. Les lectures IR et le port serie
restent disponibles en cas de defaut PID, mais un redemarrage est necessaire pour
reinitialiser le controle moteur en defaut.

## Suivi de ligne noire

Capteurs analogiques **A10 gauche, A11 milieu, A12 droite** : les broches numeriques
D10/D11/D12 ne sont pas des entrees analogiques.

La calibration par defaut est inscrite dans `src/LineFollower/line_follower.h` :

| Mesure brute | Gauche | Milieu | Droite |
|---|---:|---:|---:|
| Blanc | 28 | 28 | 26 |
| Noir | 398 | 519 | 524 |

Placer la ligne sous le detecteur central, puis presser un bumper ou envoyer `GO`.
`STOP` arrete immediatement. `BLANC` / `NOIR` permettent une nouvelle calibration
avec les trois detecteurs sur la surface correspondante, a hauteur d'utilisation.
Ces recalibrations sont en RAM ; au redemarrage, les valeurs du code sont rechargees.
`LIVE` / `PAUSE` active ou suspend l'affichage des valeurs brutes, normalisees et du mode.

Le PID de ligne corrige l'ecart optique (P=0.14, I=0.005, D=0.005). Il ne regule
pas la vitesse individuelle des roues par encodeurs dans cet environnement.
Le signal et la derivee sont filtres, la correction est adoucie pres du centre,
et les rampes d'avance et de direction sont separees. La vitesse diminue en courbe.
La croisiere reserve une marge de direction : demande actuelle 0.40, plafond roue
0.48, correction maximale 0.17, donc croisiere effective maximale 0.31.
Ces valeurs sont des commandes normalisees, pas des m/s.

En perte de ligne, le robot ralentit, tourne vers le dernier cote observe puis
cherche par pivots dans les deux sens. Il reprend le PID apres trois lectures
consecutives de la ligne, ou s'arrete apres 3 secondes sans la retrouver.
Les capteurs sont supposes places a l'avant. Les intersections ne sont pas traitees
comme des instructions de parcours.

## Capteurs de distance et servos

- Gauche A0 : **Sharp GP2D12F**, formule empirique `6787 / (ADC - 3) - 4` cm.
- Droit A3 : formule utilisateur `30 * (ADC * 0.0048828125)^(-1.173)` cm.
- Acquisition espacee de 50 ms par voie ; valeurs invalides signalees, pas converties en zero.
- Les conversions restent approximatives et dependent de la calibration et de la reference ADC.
- Servo 0 : flip du bras ; servo 1 : pouce. Limites et sens dans `src/arm.h`.

La copie locale de LibRobus n'active plus les servos dans `Robus::init()`.
La reception de telecommande IR, inutile ici et generatrice d'interruptions a 50 us,
est desactivee par defaut. Les capteurs analogiques A0/A3 restent actifs.
Le define `ROBUS_ENABLE_IR_REMOTE` permet de reactiver explicitement la telecommande.

## Organisation et validation

- `src/Control/`, `src/Motion/`, `src/robot_control.cpp` : PID, profils et odometrie.
- `src/cup.cpp` : sequence de ramassage non bloquante.
- `src/LineFollower/` : suivi PID de ligne et recuperation.
- `lib/LibRobus/` : bibliotheque robot et dependances locales.
- `test/` : verifications compile-time des retours, de la recuperation et des limites de direction.
- `archive/` : anciennes strategies, hors compilation.

Les deux environnements ont ete compiles pour la Mega. Les tests logiciels ne
remplacent pas la validation des gains, des distances et des trajectoires sur le robot.
Les sorties `.pio` et les fichiers propres au poste de developpement ne sont pas versionnes.
