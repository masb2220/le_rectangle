/**
 * @file speed_profile.cpp
 * @brief Configuration et calcul d un profil de vitesse avec acceleration et freinage.
 */

#include "Control/speed_profile.h"
#include <math.h>

/**
 * @brief Memoriser le defaut et annuler les consignes de vitesse et d acceleration.
 *
 * Conserve la derniere position du profil.
 * Le code appelant utilise le retour false pour detecter le defaut.
 *
 * @param[in,out] profile Parametres et memoire du profil.
 *
 * @return Toujours false pour signaler le defaut.
 */
static bool profile_failure(speed_profile &profile) {
    profile.fault = true; // Memoriser le defaut
    profile.vitesse = 0.0f; // Annuler la consigne de vitesse
    profile.acceleration = 0.0f; // Annuler la consigne d acceleration
    return false; // Refuser le calcul
}

/**
 * @brief Valider les parametres et initialiser un profil de vitesse.
 *
 * Valide les parametres puis remet la memoire du profil a zero.
 * Memorise les limites de vitesse, d acceleration et la periode.
 *
 * @param[in,out] profile Profil a configurer.
 * @param[in] max_speed Vitesse maximale en m/s ou rad/s.
 * @param[in] max_acceleration Acceleration maximale en m/s² ou rad/s².
 * @param[in] period Temps entre deux calculs en secondes.
 *
 * @retval true Configuration accepter.
 * @retval false Configuration refuser; configured = false et fault = true.
 *
 * @note Les champs restent modifiables. Utiliser cette fonction pour
 * changer les reglages sans contourner leur validation.
 */
bool profile_config(speed_profile &profile, float max_speed, float max_acceleration, float period) {

    // Verifier les limites et la periode avant de les memoriser
    if (!isfinite(max_speed) || max_speed <= 0.0f || !isfinite(max_acceleration) || max_acceleration <= 0.0f || !isfinite(period) || period <= 0.0f) {
        profile.configured = false; // Interdire les calculs apres un echec
        return profile_failure(profile); // Configuration refuser
    }

    profile = speed_profile{}; // Reinitialiser apres validation

    // Memoriser les parametres valider
    profile.V_MAX = max_speed; // Vitesse maximale
    profile.A_MAX = max_acceleration; // Acceleration maximale, pareil pour le freinage
    profile.period = period; // Periode en secondes
    profile.configured = true; // Autoriser la creation du profil

    return true;
}

/**
 * @brief Calculer les parametres du profil pour un deplacement demander.
 *
 * Calcule le temps pour accelerer, la vitesse maximale atteinte
 * et le temps total du deplacement.
 *
 * Si le deplacement est assez long, le profil atteint V_MAX
 * et garde cette vitesse avant de freiner.
 *
 * Si le deplacement est trop court, le profil utilise une vitesse
 * maximale plus basse et commence a freiner sans phase a vitesse constante.
 *
 * @param[in,out] profile Parametres et memoire du profil.
 * @param[in] displacement Deplacement demander, fini et signer, en m ou rad.
 *
 * @retval true Creation du profil accepter.
 * @retval false Creation refuser; fault devient true.
 *
 * @post Un calcul valide remet les consignes et le temps ecouler a zero.
 *
 * @note Configurer le profil avant d appeler cette fonction.
 * @note Le deplacement commence et termine avec une vitesse theorique nulle.
 * @note Un deplacement nul est accepter et deja terminer.
 * @note Un nouvel appel remplace le profil precedent.
 */
bool profile_start(speed_profile &profile, float displacement) {
    // Verifier l etat et l entree
    if (!profile.configured || profile.fault || !isfinite(displacement)) {
        return profile_failure(profile); // Refuser la creation du profil
    }

    const float distance = fabsf(displacement); // Deplacement sans le signe
    const float acceleration_time = profile.V_MAX / profile.A_MAX; // Temps pour atteindre la vitesse maximale

    // On calcule la distance necessaire pour accelerer jusqu a V_MAX puis freiner jusqu a zero.
    // on veut l air des deux du triangle donc 2 temps acceleration x la vitesse max.
    const float ramp_distance = profile.V_MAX * acceleration_time; // Distance totale parcouru pendant l accelerer et le freinage

    // Valider avant d utiliser ces resultats dans les prochain calculs
    if (!isfinite(acceleration_time) || acceleration_time <= 0.0f || !isfinite(ramp_distance) || ramp_distance <= 0.0f) {
        return profile_failure(profile); // Refuser les resultats invalides
    }

    // On utilise des variable temporaire pour tester
    float peak_speed = profile.V_MAX; // Vitesse maximale prevue pour ce deplacement
    float ramp_time = acceleration_time; // Temps pour accelerer/freiner
    float cruise_time = 0.0f; // Aucun temps a vitesse constante par default

    if (distance < ramp_distance) {
        // avec cette condition on cherche a savoir si juste pour accelere de 0 a vmax et freine de vmax a 0 on parcours une distance superieur au deplacement
        // Si oui, on calcule une vitesse de peak plus basse pour respecter le deplacement demander. ce sera un profil triangulaire dans ce cas
        //On cherche le temps d acceleration necessaire pour que le deplacement fait pendant la rampe sois egal au deplacement
        ramp_time = sqrtf(distance / profile.A_MAX); // Temps d acceleration reduit
        //calcul de la nouvelle vitesse de peak avec le nouveau temps d acceleration
        peak_speed = profile.A_MAX * ramp_time; // Vitesse maximale du profil triangulaire
    } else {
        // Les deux phases rentrent dans le deplacement.
        // La distance restante est parcouru a vitesse constante.
        //calcul du temps de deplacement a vitesse constante selon le deplacement restant a parcourir
        cruise_time = (distance - ramp_distance) / peak_speed; // Temps a vitesse constante
    }

    //calucul du temps de deplacement total theorique
    const float move_time = (2.0f * ramp_time) + cruise_time; // Temps total du deplacement

    // Valider avant de memoriser les parametres du profil
    if (!isfinite(peak_speed) || !isfinite(move_time) || (distance > 0.0f && (move_time <= 0.0f || peak_speed <= 0.0f))) {
        return profile_failure(profile); // Refuser les resultats invalides
    }

    // Memoriser les resultats valider
    profile.deplacement = displacement; // Deplacement total demander

    if (displacement < 0.0f) {// Sens selon le signe du deplacement
        profile.direction = -1.0f; // Deplacement negatif
    } else {
        profile.direction = 1.0f; // Deplacement positif ou nul
    }

    profile.peak_speed = peak_speed; // Vitesse maximale prevue
    profile.acceleration_time = ramp_time; // Temps de chaque phase d acceleration et de freinage
    profile.cruise_time = cruise_time; // Temps a vitesse constante
    profile.move_time = move_time; // Temps total prevu

    // initialisation des variable pour les calculs du compute
    profile.elapsed = 0.0f; // Temps ecouler depuis le debut du deplacement
    profile.position = 0.0f; // Consigne de position depuis le depart
    profile.vitesse = 0.0f; // Consigne de vitesse initiale
    profile.acceleration = 0.0f; // Consigne d acceleration initiale
    profile.started = true; // Autoriser les calculs
    profile.finished = distance == 0.0f; // Deplacement nul deja terminer

    return true;
}

/**
 * @brief Calculer les consignes du profil pour le prochain scan.
 *
 * Avance le temps du profil d une periode puis calcule les consignes :
 * - position : deplacement theorique depuis le depart.
 * - vitesse : vitesse theorique utiliser comme feedforward.
 * - acceleration : acceleration theorique de la phase actuelle.
 *
 * @par Fonctionnement
 * Le profil accelere, garde une vitesse constante si la distance le permet,
 * puis freine pour atteindre le deplacement demander avec une vitesse nulle.
 *
 * @par Validation
 * L etat du profil est verifier avant les calculs.
 * Les resultats restent temporaires jusqu a leur validation.
 * Un defaut active fault, annule vitesse et acceleration, puis retourne false.
 *
 * @param[in,out] profile Parametres et memoire du profil.
 *
 * @retval true Calcul valide ou profil deja terminer.
 * @retval false Calcul refuser; fault devient true.
 *
 * @post Un calcul valide met a jour elapsed, position, vitesse, acceleration et finished.
 *
 * @warning Le code appelant doit verifier le retour avant d utiliser les consignes.
 * Cette fonction ne commande pas directement les moteurs.
 *
 * @note Appeler a la periode configuree; la fonction ne mesure pas le temps.
 * @note Le calcul utilise le temps du profil, pas la position reel du robot.
 * @note La position est calculer directement, sans cumuler vitesse * periode.
 * @note finished indique la fin theorique du profil, pas l arrivee reel du robot.
 */
bool compute_profile(speed_profile &profile) {
    // Verifier l etat avant les calculs
    if (!profile.configured || profile.fault || !profile.started) {
        return profile_failure(profile); // Refuser le calcul
    }

    if (profile.finished) {
        //se produit lorsque le PID est pas encore stabiliser sur son SP mais le profile est terminer
        //(le PID a du retard mais on garder les consignes finales
        return true;
    }

    //puisque les profile sont utiliser comment feedforward on dois calculer les valeur du prochain cycle
    const float next_time = profile.elapsed + profile.period; // Temps du prochain scan

    // Verifier que le temps reste valide et avance vraiment
    if (!isfinite(next_time) || next_time <= profile.elapsed) {
        return profile_failure(profile); // Temps invalide ou precision insuffisante
    }

    //puisqu on calcul selon le temps theorique on dois arreter de metre a jours meme si il reste quelque cycle de PID a faire pour stabiliser,
    const float time = fminf(next_time, profile.move_time); // Ne pas depasser le temps total

    // On utilise des variables temporaires pour valider avant de memoriser.
    // Par default la vitesse et l acceleration sont nulles.
    float position = 0.0f; // Consigne de position sans le signe du deplacement
    float velocity = 0.0f; // Consigne de vitesse sans le signe du deplacement
    float acceleration = 0.0f; // Consigne d acceleration avant application du sens

    // logique de mise a jours de position, velocity et acceleration selon la phase


    //si on est arriver a la fin du profile, le position theorique = deplacement, Vitesse et acceleration restent a zero.
    if (time >= profile.move_time) {
        position = fabsf(profile.deplacement); // Consigne finale de position

    //si on se trouve dans la phase d acceleration
    } else if (time < profile.acceleration_time) {
        acceleration = profile.A_MAX; // acceleration constante
        velocity = acceleration * time; // Calcul de la vitesse theorique au temps donner
        //je multipli par 0,5 car c est plus rapide qu une division par 2
        position = 0.5f * velocity * time; // Integral theorique de la vitesse au temps donner

    //si on se trouve dans la phase de vitesse constante
    } else if (time < profile.acceleration_time + profile.cruise_time) {
        velocity = profile.peak_speed; // Phase a vitesse constante
        position = (0.5f * velocity * profile.acceleration_time) + (velocity * (time - profile.acceleration_time)); // Deplacement pendant acceleration + vitesse constante
        // Acceleration reste a zero.

    //si on se trouve dans la phase de freinage
    } else {
        const float remaining_time = profile.move_time - time; // Temps restant avant la fin
        acceleration = -profile.A_MAX; // Phase de freinage avec acceleration negative
        velocity = profile.A_MAX * remaining_time; // Vitesse theorique restante
        position = fabsf(profile.deplacement) - (0.5f * velocity * remaining_time); // Deplacement total moins distance restante
    }

    // Valider avant de memoriser les consignes
    if (!isfinite(position) || !isfinite(velocity) || !isfinite(acceleration)) {
        return profile_failure(profile); // Refuser les resultats invalides
    }

    // Memoriser les resultats valider et appliquer le sens du deplacement
    profile.elapsed = time; // Temps ecouler pour le prochain calcul
    profile.position = profile.direction * position; // Consigne de position signee
    profile.vitesse = profile.direction * velocity; // Consigne de vitesse signee
    profile.acceleration = profile.direction * acceleration; // Consigne d acceleration signee
    profile.finished = time >= profile.move_time; // Profil terminer si temps total atteint

    return true;
}
