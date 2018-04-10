#ifndef PAYLOAD_GRAVITY_PARAMS_H
#define PAYLOAD_GRAVITY_PARAMS_H
namespace jaco2_data {

struct PayloadGravityParams{
    double mass; // mass of payload in kg
    double cmx;  // payload center of mass x-component
    double cmy;  // payload center of mass y-component
    double cmz;  // payload center of mass z-component


   /* Payload Frame:
    * Top View Gripper:
    *
    *               =====|         2 Fingers
    *   z_6 ------       |xxxxx
    *             | -----|         1 Fingers
    *             |
    *             |
    *              x_6
    *
    * Distance-Wrist-Joint-Payload-Frame: l_6 = 160 mm in z_6 direction
    */
};
}

#endif // PAYLOAD_GRAVITY_PARAMS_H
