#include <Arduino.h>
#include "filters.h"

enum TRACKING_MODE {
    STATIONARY,
    TRACKING_SMOOTH,
    TRACKING_STEP
};

enum TARGET_MODE {
    TARGET_POINTER,
    TARGET_POSITION,
    TARGET_VISION,
    TARGET_NONE
};

struct rotatorCommand {
    float azm;
    float elv;
    TRACKING_MODE mode;
};

struct position {
    double lat;
    double lon;
    double alt;
};

struct pointer {
    double azm;
    double elv;
    bool isInView;
    bool isCalibrated;
};

double azimuthTo(double lat1, double lng1, double lat2, double lng2);
double distanceTo(double lat1, double lng1, double lat2, double lng2);
rotatorCommand computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2);

class rotClass {
  public:
    rotClass();
    void begin();

    void updatePointer(pointer pointerIn);

    void updatePosition(position positionIn);
    position getPosition();
    unsigned getPositionAge();

    void setGroundPosition(position positionIn);
    bool isUpdated();

    bool positionIsActive();
    bool pointerIsActive();

    void setMode(TARGET_MODE mode);
    TARGET_MODE computeMode();
    TARGET_MODE getMode();
    rotatorCommand computeCommand();

    SecondOrderEstimator latEstimator;
    SecondOrderEstimator lonEstimator;
    SecondOrderEstimator altEstimator;

    SecondOrderLowPassFilter latFilter;
    SecondOrderLowPassFilter lonFilter;
    SecondOrderLowPassFilter altFilter;

  private:
    TARGET_MODE mode;
    pointer lastPointer;
    position lastPosition;
    rotatorCommand command;
    position groundPosition;

    bool positionIsUpdated;
    bool pointerIsUpdated;

    long lastPositionTimestamp;
    long lastPointerTimestamp;
};
