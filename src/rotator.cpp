#include "rotator.h"

rotClass::rotClass() 
: latFilter(0.1, POSITION_ESTIMATION_RATE), lonFilter(0.1, POSITION_ESTIMATION_RATE), altFilter(0.5, POSITION_ESTIMATION_RATE)
{
    mode = TARGET_MODE::TARGET_NONE;
    latEstimator.setMaxTimeWindow(10000);
    lonEstimator.setMaxTimeWindow(10000);
    altEstimator.setMaxTimeWindow(10000);
    nominalPointerRate = 20;
    nominalPositionRate = 1;
};

void rotClass::begin() {
    double maxSpeed = 300;
    double maxAccel = 10;

    double CURRENT_AVG_LAT = 45.5;

    double R =  6378100;
    double dlatMaxSpeed = (180.0/PI) * (maxSpeed/R);
    double dlngMaxSpeed = (180.0/PI) * (maxSpeed/(R*cos(CURRENT_AVG_LAT*(PI/180.0))));

    double dlatMaxAccel = (180.0/PI) * (maxAccel/R);
    double dlngMaxAccel = (180.0/PI) * (maxAccel/(R*cos(CURRENT_AVG_LAT*(PI/180.0))));

    latEstimator.setMaxSpeed(dlatMaxSpeed);
    lonEstimator.setMaxSpeed(dlngMaxSpeed);
    latEstimator.setMaxAccel(dlatMaxAccel);
    lonEstimator.setMaxAccel(dlngMaxAccel);

    altEstimator.setMaxSpeed(100);
    altEstimator.setMaxAccel(10);
}

bool rotClass::isUpdated() {
    switch(mode) {
        case TARGET_MODE::TARGET_NONE:
            return true;
        break;
        case TARGET_MODE::TARGET_POINTER:
            if (pointerIsUpdated) {
                pointerIsUpdated = false;
                return true;
            }
        break;
        case TARGET_MODE::TARGET_POSITION:
        {
            // static unsigned long lastEstimationTime = millis();
            // if ((millis()-lastEstimationTime) >= 1000.0/POSITION_ESTIMATION_RATE) {
            //     lastEstimationTime = millis();
            //     return true;
            // }
            if (positionIsUpdated) {
                positionIsUpdated = false;
                return true;
            }
        }
        break;
    } 
    return false;     
};

void rotClass::updatePosition(position positionIn) {
    if (positionIn.lat != lastPosition.lat or positionIn.lon != lastPosition.lon or positionIn.alt != lastPosition.alt) {
        positionIsUpdated = true;
        lastPositionTimestamp = millis();
        lastPosition = positionIn;
    }
};

void rotClass::setGroundPosition(position positionIn) {
    groundPosition = positionIn;
};

void rotClass::updatePointer(pointer pointerIn) {
    pointerIsUpdated = true;
    lastPointerTimestamp = millis();
    lastPointer = pointerIn;
};

bool rotClass::positionIsActive() {
    if (millis() - lastPositionTimestamp > 2*(1000/nominalPositionRate)) {
        latEstimator.reset();
        lonEstimator.reset();
        altEstimator.reset();
        return false;
    }
    else {
        return true;
    }
}

bool rotClass::pointerIsActive() {
    if (millis() - lastPointerTimestamp > 2*(1000/nominalPointerRate)) {
        return false;
    }
    else {
        return true;
    }
}

TARGET_MODE rotClass::getMode() {
    return mode;
};

void rotClass::setMode(TARGET_MODE modeIn) {
    mode = modeIn;
};

rotatorCommand rotClass::computeCommand() {
    rotatorCommand commandOut;
    switch (mode) {
        case TARGET_NONE:
            commandOut.azm = 0;
            commandOut.elv = 0;
            commandOut.mode = TRACKING_MODE::STATIONARY;
        break;
        case TARGET_POINTER:
            commandOut.azm = lastPointer.azm;
            commandOut.elv = lastPointer.elv;
            commandOut.mode = TRACKING_MODE::TRACKING_SMOOTH;
        break;
        case TARGET_POSITION:
            {
            // // WARNING -- ONLY FOR TESTING!!!
            // groundPosition.lat = 46.1457158;
            // groundPosition.lon = 6.1874207;
            // groundPosition.alt = 1379;
            // // WARNING -- ONLY FOR TESTING!!!

            double latEstimation = latEstimator.computeAngle(millis());
            double lonEstimation = lonEstimator.compute(millis());
            double altEstimation = altEstimator.compute(millis());

            // From now on all angle values are in the referential of the tracker, not north referenced

            double latFiltered = latFilter.process(latEstimation);
            double lonFiltered = lonFilter.process(lonEstimation);
            double altFiltered = altFilter.process(altEstimation);

            rotatorCommand positionRaw;
            rotatorCommand positionFiltered;

            positionFiltered = computeAngle(groundPosition.lat, groundPosition.lon, groundPosition.alt, latFiltered, lonFiltered, altFiltered);
            positionRaw = computeAngle(groundPosition.lat, groundPosition.lon, groundPosition.alt, lastPosition.lat, lastPosition.lon, lastPosition.alt);

            commandOut.mode = TRACKING_MODE::TRACKING_SMOOTH;
            }
        break;
    } 
    
    return commandOut;
};

TARGET_MODE rotClass::computeMode() {
    if (lastPointer.isInView and lastPointer.isCalibrated) {
        return TARGET_MODE::TARGET_POINTER;
    } 
    else if (positionIsActive()) {
        return TARGET_MODE::TARGET_POSITION;
    }
    else {
        return TARGET_MODE::TARGET_NONE;
    }
}

position rotClass::getPosition() {
    return lastPosition;
}

unsigned rotClass::getPositionAge() {
    return millis()-lastPositionTimestamp;
}

double azimuthTo(double lat1, double lng1, double lat2, double lng2) {
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlon = lng2-lng1;
  double a1 = sin(dlon) * cos(lat2);
  double a2 = sin(lat1) * cos(lat2) * cos(dlon);
  a2 = cos(lat1) * sin(lat2) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += 2*PI;
  }
  return a2/PI*180.0;
}

double distanceTo(double lat1, double lng1, double lat2, double lng2) {
  double R = 6371000;
  lat1 = lat1 * PI / 180.0;
  lng1 = lng1 * PI / 180.0;
  lat2 = lat2 * PI / 180.0;
  lng2 = lng2 * PI / 180.0;

  double dlat = lat2-lat1;
  double dlng = lng2-lng1;

  double a = sin(dlat/2) * sin(dlat/2) + cos(lat1) * cos(lat2) * sin(dlng/2) * sin(dlng/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double d = R * c;
  return d;
}

rotatorCommand computeAngle(double lat1, double lng1, double alt1, double lat2, double lng2, double alt2) {
    rotatorCommand command;
    command.azm = azimuthTo(lat1, lng1, lat2, lng2);
    command.elv = (atan((alt2 - alt1) / distanceTo(lat1, lng1, lat2, lng2))/PI)*180.0;
    return command;
}


