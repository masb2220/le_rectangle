#pragma once

constexpr float CUP_PI = 3.14159265359f;
constexpr float cupReturnDistance(float signedPickupMetres)
{
    return -signedPickupMetres;
}
// Les deux orientations proviennent de l'odometrie et sont dans [-pi, pi].
constexpr float cupReturnAngle(float initialTheta, float currentTheta)
{
    return initialTheta-currentTheta > CUP_PI ? initialTheta-currentTheta-2*CUP_PI :
        initialTheta-currentTheta < -CUP_PI ? initialTheta-currentTheta+2*CUP_PI :
        initialTheta-currentTheta;
}
