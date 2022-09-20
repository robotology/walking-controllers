namespace yarp WalkingControllers.YarpUtilities

/**
 * Representation of a 3D vector
 */
struct Vector3 {
    1: double x;
    2: double y;
    3: double z;
}

/**
 * Representation of a Quaternion
 */
struct Quaternion {
    1: double w;
    2: Vector3 imaginary;
}

/**
 * Representation of the IHumanState interface
 */
struct HumanState {
    1: list<string> jointNames;
    2: list<double> positions;
    3: list<double> velocities;

    4: string baseName;
    5: Vector3 baseOriginWRTGlobal;
    6: Quaternion baseOrientationWRTGlobal;
    7: list<double> baseVelocityWRTGlobal;
    8: Vector3 CoMPositionWRTGlobal;
    9: Vector3 CoMVelocityWRTGlobal;
}