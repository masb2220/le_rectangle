/**
 * @file robot_control.cpp
 * @brief Configuration, acquisition et execution de la chaine de controle.
 */

#include "robot_control.h"
#include "robot_serial.h"

#include <Arduino.h>
#include <LibRobus.h>
#include <math.h>
#include <stdint.h>

#include "Control/PID_controller.h"
#include "tuning/wheel_motor_calibration.h"
#include "Control/speed_profile.h"
#include "Control/Inverse_differential_kinematics.h"
#include "Control/wheel_speed.h"
#include "Control/odometry.h"
#include "Motion/motion_sequencer.h"

#define ROBOT_B
// Cadence
static constexpr unsigned long CONTROL_PERIOD_US = 10000UL;
static constexpr float CONTROL_PERIOD_S = CONTROL_PERIOD_US / 1000000.0f; // Secondes, derivees de la cadence.
static constexpr float PI_VALUE = 3.14159265358979323846f; // Valeur de pi pour les conversions angulaires.

// Positif = compensation vers la gauche; zero = desactive.

#ifdef ROBOT_A
static constexpr float LATERAL_CORRECTION_MM_PER_M = -40.0f;
static constexpr float REVERSE_LATERAL_CORRECTION_MM_PER_M = 0.0f;
static constexpr float WHEEL_SPACING_M = 0.172; // Entraxe (pas la bacterie)   ///si tourne pas assez augmenter cette valeur
#endif

#ifdef ROBOT_B
static constexpr float LATERAL_CORRECTION_MM_PER_M = 45.0f;
static constexpr float REVERSE_LATERAL_CORRECTION_MM_PER_M = 0.0f;
static constexpr float WHEEL_SPACING_M = 0.1735; // Entraxe (pas la bacterie)
#endif


// Parametres du modele
static constexpr float WHEEL_DIAMETER_M = 0.0765f; // Diametre
static constexpr float GEARBOX_RATIO = 50.0f; // Rapport moteur / roue
static constexpr uint32_t ENCODER_COUNTS_PER_TURN = 64; // Comptes par tour moteur

// Marges communes : calibration -> commandes roues -> profils.
static constexpr float WHEEL_SPEED_USAGE = 0.95f; // Reserver 5 % de la vitesse estimee pour le PI moteur.
static constexpr float TRANSLATION_TEST_SPEED_SCALE = 1.0f;

static constexpr float ROTATION_TEST_SPEED_SCALE = 1.0f;
 // Test comparatif : 1.0f pour retrouver la vitesse normale.
static constexpr float PROFILE_SPEED_USAGE = 0.90f; // Reserver 10 % pour les corrections de suivi.
static_assert(WHEEL_SPEED_USAGE > 0.0f && WHEEL_SPEED_USAGE < 1.0f, "Marge moteur invalide");
static_assert(PROFILE_SPEED_USAGE > 0.0f && PROFILE_SPEED_USAGE < 1.0f, "Marge profil invalide");

// Activation materielle
static constexpr bool HARDWARE_CONNECTED = true; // Garder false sur Mega seule
static bool hardware_initialized = false; // Initialisation du hardware LibRobus (mode de test)

// Controleurs des roues
static PID_controller left_pid; // PI gauche
static PID_controller right_pid; // PI droit

// Controleurs du mouvement
static PID_controller position_pid; // Correction de position
static PID_controller angle_pid; // Correction d'orientation

// Les profils lineaire et angulaire sont garder dans le sequenceur motion.

// Consignes et conversion des mesures
static wheel_setpoint wheel_commands; // Consignes gauche et droite
static wheel_speed_config left_wheel; // Parametres roue gauche
static wheel_speed_config right_wheel; // Parametres roue droite

// Position et mouvement
static odometry robot; // Position estimee
static motion_sequencer motion; // Sequenceur de mouvement

// Memoire des encodeurs
static int32_t previous_left_count = 0; // Lecture precedente gauche
static int32_t previous_right_count = 0; // Lecture precedente droite

// Commandes moteur
static float left_motor_command = 0.0f; // Commande gauche
static float right_motor_command = 0.0f; // Commande droite

// Etat global
static unsigned long last_control_time = 0; // Derniere echeance
static ControlFault first_fault = ControlFault::NONE;
static unsigned long fault_interval_us = 0;
static bool system_fault = false; // Defaut memorise
static bool control_initialized = false; // Initialisation globale terminee
#ifdef MOTION_DIAGNOSTIC
extern void diagnostic_wheel_sample(float, float, float, float, float, float);
#endif
#ifdef ENCODER_AUDIT
extern void encoder_audit_sample(unsigned long, int32_t, int32_t, int32_t, int32_t,
                                 float, float, float, float, float, float, float);
#endif
static bool rotation_speed_overridden = false;
static float saved_rotation_speed = 0.0f;
static bool straight_motion_overridden = false;
static float saved_linear_speed = 0.0f;
static float saved_lateral_gain = 0.0f;

static void restore_motion_overrides() {
    if (straight_motion_overridden) {
        motion.movement.linear_profile.V_MAX = saved_linear_speed;
        motion.movement.lateral_gain = saved_lateral_gain;
        straight_motion_overridden = false;
    }
    if (rotation_speed_overridden) {
        motion.movement.angular_profile.V_MAX = saved_rotation_speed;
        rotation_speed_overridden = false;
    }
}

/**
 * @brief Arret des moteurs
 *
 * Mettre les deux commande moteur memoriser a zero.
 * Envoie aussi zero avec LibRobus si le materiel a ete initialiser.
 * Ne remet pas a zero les PID et ne mesure pas si les roues sont physiquement arreter.
 */
static void stop_motors() {
    left_motor_command = 0.0f; // Annuler la commande gauche
    right_motor_command = 0.0f; // Annuler la commande droite

    if (hardware_initialized) {
        MOTOR_SetSpeed(LEFT, 0.0f); // Arreter le moteur gauche
        MOTOR_SetSpeed(RIGHT, 0.0f); // Arreter le moteur droit
    }
}

/**
 * @brief Memorisation d'un defaut global
 *
 * Verrouiller le controle en defaut et demander l arret des deux moteur.
 * Les prochain appels update ne font plus de cycle tant que le defaut reste present.
 */
static void set_system_fault(ControlFault reason) {
    if(first_fault == ControlFault::NONE) first_fault = reason;
    system_fault = true; // Bloquer les prochains cycles
    stop_motors(); // Arreter les commandes moteur
    restore_motion_overrides();
}

/**
 * @brief Configuration des PID
 *
 * Configurer les quatre instance de PID avec les gains configurer du robot.
 * Les deux PI de roue transforme une erreur en m/s en commande moteur entre -1 et 1.
 * Le PID position produit des m/s et le PID angle produit des rad/s.
 * Retour true seulement si les quatre configuration sont accepter.
 *
 * @return true seulement si les quatre configuration sont accepter.
 */
static bool configure_controllers(float max_linear_speed, float max_angular_speed) {
    // Gains valider roues libres avec feedforward moteur
    const bool left_ok = PID_config(left_pid, WHEEL_KP[0], WHEEL_KI[0], 0.0f, -WHEEL_COMMAND_LIMIT, WHEEL_COMMAND_LIMIT, CONTROL_PERIOD_S);

    // Gains valider roues libres avec feedforward moteur
    const bool right_ok = PID_config(right_pid, WHEEL_KP[1], WHEEL_KI[1], 0.0f, -WHEEL_COMMAND_LIMIT, WHEEL_COMMAND_LIMIT, CONTROL_PERIOD_S);

    // Position vers vitesse en m/s
    const bool position_ok = PID_config( position_pid, 10.0f, 0.0f, 0.0f, -max_linear_speed, max_linear_speed, CONTROL_PERIOD_S); // Limite commune derivee de la calibration

    // Angle vers rotation en rad/s
    const bool angle_ok = PID_config( angle_pid, 10.0f, 0.0f, 0.0f, -max_angular_speed, max_angular_speed, CONTROL_PERIOD_S);

    return left_ok && right_ok && position_ok && angle_ok;
}

/**
 * @brief Configuration des autres modules
 *
 * Initialiser tous les module qui seront utiliser dans la chaine de controle.
 * Configure PID, profils, cinematique, conversion encodeur, odometrie et sequenceur.
 * Les periode utilise sont les meme pour que tous les calcul suivent le meme scan.
 * Retour true seulement si chaque module est configurer; ne demarre aucun mouvement.
 *
 * @return true seulement si chaque module est configurer; ne demarre aucun mouvement.
 */
static bool configure_modules() {
    const float max_wheel_speed = calibrated_wheel_speed_limit() * WHEEL_SPEED_USAGE;
    const float max_angular_speed = 2.0f * max_wheel_speed / WHEEL_SPACING_M;
    if (!isfinite(max_wheel_speed) || max_wheel_speed <= 0.0f || !isfinite(max_angular_speed) || max_angular_speed <= 0.0f) {
        return false; // Refuser une calibration inexploitable avant de configurer les modules.
    }
    // Ces limites representent une translation pure ou une rotation pure.
    // Si les deux sont combinees, la cinematique reduit les consignes proportionnellement.
    const bool controllers_ok = configure_controllers(max_wheel_speed, max_angular_speed);

    // Acceleration en m/s²
    const bool linear_ok = profile_config(motion.movement.linear_profile, max_wheel_speed * PROFILE_SPEED_USAGE * TRANSLATION_TEST_SPEED_SCALE, 0.6f, CONTROL_PERIOD_S);

    // Acceleration en rad/s²
    const bool angular_ok = profile_config(motion.movement.angular_profile, max_angular_speed * PROFILE_SPEED_USAGE * ROTATION_TEST_SPEED_SCALE, 8.5f, CONTROL_PERIOD_S);

    // Conversion vers les roues
    const bool kinematic_ok = Kinematic_config( wheel_commands, WHEEL_SPACING_M, max_wheel_speed);

    // Conversion encodeur gauche
    const bool left_wheel_ok = configure_wheel_speed( left_wheel, ENCODER_COUNTS_PER_TURN, GEARBOX_RATIO, WHEEL_DIAMETER_M, CONTROL_PERIOD_S);

    // Conversion encodeur droit
    const bool right_wheel_ok = configure_wheel_speed( right_wheel, ENCODER_COUNTS_PER_TURN, GEARBOX_RATIO, WHEEL_DIAMETER_M, CONTROL_PERIOD_S);

    // Position initiale estimee du robot
    const bool odometry_ok = odometry_config( robot, WHEEL_SPACING_M, CONTROL_PERIOD_S, 0.0f, 0.0f, PI_VALUE / 2.0f);

    motion.movement.forward_lateral_correction_m_per_m = LATERAL_CORRECTION_MM_PER_M / 1000.0f;
    motion.movement.reverse_lateral_correction_m_per_m = REVERSE_LATERAL_CORRECTION_MM_PER_M / 1000.0f;

    // Sequenceur au repos
    const bool motion_ok = motion_config( motion, CONTROL_PERIOD_S);

    return controllers_ok && linear_ok && angular_ok
        && kinematic_ok && left_wheel_ok && right_wheel_ok
        && odometry_ok && motion_ok;
}

/**
 * @brief Initialisation de LibRobus
 *
 * Initialiser LibRobus puis demander des commande moteur nulle.
 * Lit les encodeur pour memoriser le point de depart des prochain delta.
 * hardware_initialized indique que l appel a ete fait, pas une detection de presence du robot.
 */
static void initialize_hardware() {
    BoardInit(); // Initialiser les peripheriques du robot
    hardware_initialized = true; // Appel execute, sans detection de presence
    stop_motors(); // Demarrer avec des commandes nulles

    previous_left_count = ENCODER_Read(LEFT); // Reference initiale gauche
    previous_right_count = ENCODER_Read(RIGHT); // Reference initiale droite
}

/**
 * @brief Lecture des variations des compteurs
 *
 * Lire les compteurs cumuler et calculer les comptes depuis le dernier scan.
 * Les deux argument avec & sont des sortie : la fonction ecrit les delta dedans.
 * Soustrait sur 64 bits avant de verifier que chaque resultat entre dans un int32_t.
 * Memorise les nouvelle lecture seulement si les deux difference sont acceptable.
 * Retour true si traite; false si indisponible ou hors plage. Ne valide pas la presence des encodeur.
 *
 * @param[out] left_delta_count Variation signee des comptes de la roue gauche.
 * @param[out] right_delta_count Variation signee des comptes de la roue droite.
 *
 * @return true si traite; false si indisponible ou hors plage. Ne valide pas la presence des encodeur.
 */
static bool read_encoder_changes(int32_t &left_delta_count, int32_t &right_delta_count) {
    left_delta_count = 0; // Valeur par defaut
    right_delta_count = 0;

    if (!hardware_initialized || system_fault) {
        return false; // Acquisition non autorisee
    }

    const int32_t left_count = ENCODER_Read(LEFT); // Lecture gauche
    const int32_t right_count = ENCODER_Read(RIGHT); // Lecture droite

    const int64_t left_difference =
        static_cast<int64_t>(left_count) - previous_left_count; // Calcul sur 64 bits

    const int64_t right_difference =
        static_cast<int64_t>(right_count) - previous_right_count; // Calcul sur 64 bits

    if ((left_difference < INT32_MIN) || (left_difference > INT32_MAX) || (right_difference < INT32_MIN) || (right_difference > INT32_MAX)) {
        return false; // Variation hors plage
    }

    left_delta_count = static_cast<int32_t>(left_difference); // Variation gauche
    right_delta_count = static_cast<int32_t>(right_difference); // Variation droite

    previous_left_count = left_count; // Memoriser les lectures
    previous_right_count = right_count;

    return true; // Lectures traitees
}

/**
 * @brief Chaine de calcul du controle
 *
 * Faire la chaine de calcul pour un scan a partir des deux delta encodeur.
 * Ordre : vitesse mesurer -> odometrie -> sequenceur avec profils -> consigne roue -> PI roue.
 * Les mesure de roue servent au retour des PI; les consigne viennent de la cinematique.
 * Memorise les deux commande moteur finale seulement si la chaine est valide.
 * Retour false des qu un module echoue; cette fonction n envoie rien au moteur.
 *
 * @param[in] left_delta_count Variation signee des comptes de la roue gauche.
 * @param[in] right_delta_count Variation signee des comptes de la roue droite.
 *
 * @return false des qu un module echoue; cette fonction n envoie rien au moteur.
 */
static bool compute_control(int32_t left_delta_count, int32_t right_delta_count) {
#ifdef PERFORMANCE_BENCH
    performance_mark(0); // Instrumentation uniquement dans le benchmark Mega seule
#endif
    left_motor_command = 0.0f; // Commandes nulles par defaut
    right_motor_command = 0.0f;

    const float left_speed =
        compute_wheel_speed(left_delta_count, left_wheel); // Mesure gauche

    const float right_speed =
        compute_wheel_speed(right_delta_count, right_wheel); // Mesure droite

    if (!isfinite(left_speed) || !isfinite(right_speed)) {
        return false; // Vitesses invalides
    }

#ifdef PERFORMANCE_BENCH
    performance_mark(1);
#endif
    if (!compute_odometry(robot, left_speed, right_speed)) {
        return false; // Defaut d'odometrie
    }

#ifdef PERFORMANCE_BENCH
    performance_mark(2);
#endif
    if (!compute_motion(motion, robot, position_pid, angle_pid)) {
        return false; // Defaut du sequenceur
    }

#ifdef PERFORMANCE_BENCH
    performance_mark(3);
#endif
    // Au repos, couper la commande et effacer la memoire des PI de roues.
    // L acquisition et l odometrie restent actives pendant l attente du bumper.
    if (motion.state == MotionState::FINISHED) {
        left_pid.integrale = 0.0f;
        left_pid.last_error = 0.0f;
        left_pid.init = false;
        right_pid.integrale = 0.0f;
        right_pid.last_error = 0.0f;
        right_pid.init = false;
        wheel_commands.left = 0.0f;
        wheel_commands.right = 0.0f;
        return true; // Les commandes moteur ont deja ete mises a zero en debut de calcul.
    }

    const float linear_command = motion.movement.linear_speed_cmd; // Profil + correction position en m/s
    const float angular_command = motion.movement.angular_speed_cmd; // Profil + correction angle en rad/s

    if (!compute_wheel_speed_setpoints( wheel_commands, linear_command, angular_command)) {
        return false; // Defaut de cinematique
    }

#ifdef PERFORMANCE_BENCH
    performance_mark(4);
#endif
    const float left_command =
        compute_PID(left_pid, wheel_commands.left, left_speed, calibrated_wheel_feedforward(0, wheel_commands.left)); // PI + FF gauche, saturation commune

    const float right_command =
        compute_PID(right_pid, wheel_commands.right, right_speed, calibrated_wheel_feedforward(1, wheel_commands.right)); // PI + FF droit, saturation commune

    if (left_pid.fault || right_pid.fault || !isfinite(left_command) || !isfinite(right_command)) {
        return false; // Defaut des PI
    }

#ifdef PERFORMANCE_BENCH
    performance_mark(5);
#endif
    left_motor_command = left_command; // Memoriser les commandes valides
    right_motor_command = right_command;
#ifdef MOTION_DIAGNOSTIC
    diagnostic_wheel_sample(wheel_commands.left, wheel_commands.right,
                            left_speed, right_speed, left_command, right_command);
#endif

    return true;
}

/**
 * @brief Application des commandes moteur
 *
 * Verifier les deux commande moteur avant de les transmettre a LibRobus.
 * Accepte des valeur fini entre -1 et 1, seulement si initialiser et sans defaut global.
 * Retour false avec demande d arret si invalide.
 * Retour true signifie commande transmise, pas confirmation que le moteur tourne.
 *
 * @return false avec demande d arret si invalide.
 */
static bool apply_motor_commands() {
    if (!hardware_initialized || system_fault) {
        stop_motors(); // Interdire les commandes
        return false;
    }

    if (!isfinite(left_motor_command) || !isfinite(right_motor_command) || (left_motor_command < -1.0f) || (left_motor_command > 1.0f) || (right_motor_command < -1.0f) || (right_motor_command > 1.0f)) {
        stop_motors(); // Commande invalide
        return false;
    }

    MOTOR_SetSpeed(LEFT, left_motor_command); // Commande gauche
    MOTOR_SetSpeed(RIGHT, right_motor_command); // Commande droite

    return true; // Commandes transmises, sans retour materiel
}

/**
 * @brief Execution d'un cycle
 *
 * Executer une acquisition, les calcul de controle, puis la commande des moteur.
 * Si une etape echoue, memorise un defaut global et demande l arret.
 * Cette fonction fait un seul scan; la cadence est gerer par robot_control_update.
 */
static void control_cycle() {
    int32_t left_delta_count = 0; // Variation gauche
    int32_t right_delta_count = 0; // Variation droite

    if (!read_encoder_changes(left_delta_count, right_delta_count)) {
        set_system_fault(ControlFault::ACQUISITION); // Defaut d'acquisition
        return;
    }

    if (!compute_control(left_delta_count, right_delta_count)) {
        set_system_fault(ControlFault::CALCULATION); // Defaut de calcul
        return;
    }

#ifdef ENCODER_AUDIT
    encoder_audit_sample(micros(), previous_left_count, previous_right_count,
        left_delta_count, right_delta_count, robot.x, robot.y, robot.theta,
        left_wheel.speed_per_count * CONTROL_PERIOD_S, WHEEL_SPACING_M, 0.0f, 0.0f);
#endif
    if (!apply_motor_commands()) {
        set_system_fault(ControlFault::MOTOR_COMMAND); // Defaut de commande
    }
}

/**
 * @brief Initialisation appelee depuis setup()
 *
 * Point d entrer appeler une fois depuis setup pour preparer tout le controle.
 * Configure les module et initialise LibRobus seulement si HARDWARE_CONNECTED vaut true.
 * Demarre la reference de temps et retourne true si l initialisation est accepter.
 * Retour false avec defaut si la config echoue. Ne lance pas de mouvement vers une cible.
 *
 * @return false avec defaut si la config echoue. Ne lance pas de mouvement vers une cible.
 */
bool robot_control_init() {
    first_fault = ControlFault::NONE;
    fault_interval_us = 0;
    restore_motion_overrides();
    control_initialized = false; // Bloquer pendant l'initialisation
    stop_motors(); // Arreter avant une eventuelle reinitialisation

    if (!configure_modules()) {
        set_system_fault(ControlFault::CONFIGURATION); // Configuration refusee
        return false;
    }

    system_fault = false; // Configuration acceptee

    if (HARDWARE_CONNECTED) {
        initialize_hardware(); // Initialiser le robot
    }
    else {
        robot_serial_message("Materiel desactive"); // Fonctionnement sur Mega seule
    }

    last_control_time = micros(); // Depart de la cadence
    control_initialized = true; // Autoriser les appels periodiques
    robot_serial_message("Controle : trajectoire en S (V3), odometrie encodeurs");

    return true;
}

/**
 * @brief Fonction appelee continuellement depuis loop()
 *
 * Point d entrer appeler souvent depuis loop, sans delay.
 * Retourne directement si pas pret, en defaut ou materiel desactiver.
 * Compare micros a la derniere echeance : attend 10 ms puis execute un cycle.
 * Si le retard atteint deux periode, memorise un defaut et demande l arret.
 * Avance l echeance d une periode fixe : ce code ne mesure pas le dt reel pour les calcul.
 * Ce n est pas une interruption; la frequence des appels depend aussi du reste de loop.
 */
void robot_control_update() {
    if (!control_initialized || system_fault || !HARDWARE_CONNECTED) {
        return; // Aucun cycle sans initialisation valide
    }

    const unsigned long now = micros(); // Temps actuel
    const unsigned long elapsed = now - last_control_time; // Temps ecoule

    if (elapsed < CONTROL_PERIOD_US) {
        return; // Echeance pas encore atteinte
    }

    if (elapsed >= (2UL * CONTROL_PERIOD_US)) {
        fault_interval_us = elapsed;
        set_system_fault(ControlFault::MISSED_PERIOD); // Une periode complete a ete manquee
        robot_serial_message("DEFAUT : periode de controle manquee");
        return;
    }

    last_control_time += CONTROL_PERIOD_US; // Maintenir la cadence
    control_cycle(); // Executer acquisition, calculs et commandes
    if (motion.state == MotionState::FINISHED) restore_motion_overrides();
}

unsigned long robot_control_background_time_us() {
    if (!control_initialized || system_fault) return 0;
    const unsigned long elapsed = micros() - last_control_time;
    return elapsed < CONTROL_PERIOD_US ? CONTROL_PERIOD_US - elapsed : 0;
}

bool robot_control_start_rotation(float delta_angle, float speed_limit) {
    if (!robot_control_finished() || rotation_speed_overridden || !isfinite(delta_angle) ||
        !isfinite(speed_limit) || speed_limit <= 0 ||
        speed_limit > motion.movement.angular_profile.V_MAX) return false;
    saved_rotation_speed = motion.movement.angular_profile.V_MAX;
    motion.movement.angular_profile.V_MAX = speed_limit;
    rotation_speed_overridden = true;
    if (!robot_control_start_relative(0.0f, delta_angle)) {
        restore_motion_overrides();
        return false;
    }
    return true;
}

/**
 * @brief Demander un mouvement vers une cible absolue si le controle est disponible.
 *
 * Transmettre une cible au sequenceur seulement si le robot est pret.
 *
 * @param[in] target_x Position cible absolue X, en metres.
 * @param[in] target_y Position cible absolue Y, en metres.
 * @param[in] target_angle Orientation cible, en radians.
 * @param[in] direction Sens de translation : +1 ou -1.
 *
 * @return True si accepter, false si indisponible, occuper ou invalide.
 */
bool robot_control_start(float target_x, float target_y, float target_angle, int direction) {
    if (!control_initialized || system_fault || !hardware_initialized) {
        return false; // Robot indisponible
    }

    return motion_start(motion, target_x, target_y, target_angle, direction);
}

bool robot_control_start_relative(float distance, float delta_angle) {
    if (!control_initialized || system_fault || !hardware_initialized) { return false; }
    return motion_start_relative(motion, robot, distance, delta_angle);
}

bool robot_control_start_straight(float distance, float speed_limit) {
    if (!robot_control_finished() || rotation_speed_overridden || straight_motion_overridden ||
        !isfinite(distance) || distance == 0.0f || !isfinite(speed_limit) || speed_limit <= 0.0f) return false;

    saved_linear_speed = motion.movement.linear_profile.V_MAX;
    saved_lateral_gain = motion.movement.lateral_gain;
    straight_motion_overridden = true;
    motion.movement.linear_profile.V_MAX = fminf(speed_limit, saved_linear_speed);
    motion.movement.lateral_gain = 0.0f; // PID de cap fixe uniquement.

    // La demande relative construit sa cible sans le decalage lateral calibre.
    const float forward = motion.movement.forward_lateral_correction_m_per_m;
    const float reverse = motion.movement.reverse_lateral_correction_m_per_m;
    motion.movement.forward_lateral_correction_m_per_m = 0.0f;
    motion.movement.reverse_lateral_correction_m_per_m = 0.0f;
    const bool accepted = robot_control_start_relative(distance, 0.0f);
    motion.movement.forward_lateral_correction_m_per_m = forward;
    motion.movement.reverse_lateral_correction_m_per_m = reverse;
    if (!accepted) restore_motion_overrides();
    return accepted;
}
bool robot_control_get_pose(float &x, float &y, float &theta) {
    if (!control_initialized || system_fault || robot.fault || motion.movement.fault) { return false; }
    x = robot.x;
    y = robot.y;
    theta = robot.theta;
    return true;
}

MotionFault robot_control_motion_fault() {
    return motion.movement.fault_reason;
}

/**
 * @brief Annuler le mouvement et verrouiller le controle avec des commandes moteur nulles.
 *
 * Annuler le mouvement et verrouiller les commandes moteur.
 */
void robot_control_stop() {
    motion.state = MotionState::FINISHED; // Annuler le mouvement
    motion.movement.linear_speed_cmd = 0.0f; // Annuler la translation
    motion.movement.angular_speed_cmd = 0.0f; // Annuler la rotation
    motion.movement.profile_initialized = false; // Abandonner le profil

    set_system_fault(ControlFault::STOP_REQUESTED); // Demander zero aux moteurs et bloquer les prochain cycles
}

/**
 * @brief Lire si le sequenceur est dans son etat termine.
 *
 * Indiquer si le sequenceur est au repos.
 *
 * @return True si l etat est FINISHED; consulter aussi le defaut global.
 */
bool robot_control_finished() {
    return control_initialized && !system_fault && !motion.movement.fault && motion.state == MotionState::FINISHED;
}

/**
 * @brief Lire le defaut global du controle.
 *
 * Indiquer si le controle est en defaut.
 *
 * @return True si le controle est verrouiller en defaut.
 */
bool robot_control_fault() {
    return system_fault || motion.movement.fault;
}

/**
 * @brief Afficher la position estimee et l etat du controle au repos.
 *
 * Afficher seulement au repos pour pas perturber le controle pendant un mouvement.
 *
 * @param[in,out] output Flux recevant le statut; utiliser au repos pour eviter le formatage pendant le controle.
 */
void robot_control_print_status(Print &output) {
    output.print("Etat : ");
    output.println(static_cast<int>(motion.state));

    output.print("X : ");
    output.print(robot.x, 3);
    output.print(" Y : ");
    output.print(robot.y, 3);
    output.print(" Angle : ");
    output.println(robot.theta, 3);

    output.print("Defaut : ");
    output.println(system_fault ? "OUI" : "NON");
    output.print("Defaut mouvement (0=aucun, 1=etat, 2=delai, 3=position) : ");
    output.println(static_cast<int>(motion.movement.fault_reason));

    output.print("Materiel initialise : ");
    output.println(hardware_initialized ? "OUI" : "NON");
}

ControlFault robot_control_fault_reason() { return first_fault; }
unsigned long robot_control_fault_interval_us() { return fault_interval_us; }


void robot_control_cancel() {
    if (!control_initialized || system_fault) return;
    motion.state = MotionState::FINISHED;
    reset_motion(motion.movement, position_pid, angle_pid);
    left_pid.integrale = right_pid.integrale = 0.0f;
    left_pid.init = right_pid.init = false;
    stop_motors();
    restore_motion_overrides();
}
