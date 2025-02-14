/*
   INDI Developers Manual
   Tutorial #7

   "Simple telescope simulator"

   We construct a most basic (and useless) device driver to illustrate INDI.

   Refer to README, which contains instruction on how to build this driver, and use it
   with an INDI-compatible client.

*/

#include "./SG1.h"

#include "indicom.h"

#include <memory>

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

using namespace INDI::AlignmentSubsystem;

// We declare an auto pointer to SG1.
std::unique_ptr<SG1> telescope_sim(new SG1());

SG1::SG1() : DBG_SG1(INDI::Logger::getInstance().addDebugLevel("SG1 status","SG1"))
{
}

int openSerialPort(const char* portname)
{
    int fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        return -1;
    }
    return fd;
}

bool configureSerialPort(int fd, int speed)
{
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag
        = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit characters
    tty.c_iflag &= ~IGNBRK; // disable break processing
    tty.c_lflag = 0; // no signaling chars, no echo, no
                     // canonical processing
    tty.c_oflag = 0; // no remapping, no delays
    tty.c_cc[VMIN] = 1; // read doesn't block
    tty.c_cc[VTIME] = 0; // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF
                     | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag
        |= (CLOCAL | CREAD); // ignore modem controls,
                             // enable reading
    tty.c_cflag &= ~(PARENB | PARODD); // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        return false;
    }
    return true;
}

int readFromSerialPort(int fd, char* buffer, size_t size)
{
    return read(fd, buffer, size);
}

int writeToSerialPort(int fd, const char* buffer,
    size_t size)
{
    return write(fd, buffer, size);
}

void closeSerialPort(int fd) { close(fd); }

bool SG1::Abort()
{
    if (MovementNSSP.getState() == IPS_BUSY)
    {
        MovementNSSP.reset();
        MovementNSSP.setState(IPS_IDLE);
        MovementNSSP.apply();
    }

    if (MovementWESP.getState() == IPS_BUSY)
    {
        MovementWESP.setState(IPS_IDLE);
        MovementWESP.reset();
        MovementWESP.apply();
    }

    if (EqNP.getState() == IPS_BUSY)
    {
        EqNP.setState(IPS_IDLE);
        EqNP.apply();
    }

    TrackState = SCOPE_IDLE;

    AxisStatusRA = AxisStatusDEC = STOPPED; // This marvelous inertia free scope can be stopped instantly!

    AbortSP.setState(IPS_OK);
    AbortSP.reset();
    AbortSP.apply();
    LOG_INFO("Telescope aborted.");

    return true;
}

bool SG1::Connect()
{
    SerialPort = openSerialPort("/dev/ttyUSB0");
    if(SerialPort == -1){
        return false;
    }

    if(!configureSerialPort(SerialPort,9600)){
        return false;
    }

    SetTimer(200);
    return true;
}

bool SG1::Disconnect()
{
    return true;
}

const char *SG1::getDefaultName()
{
    return (const char *)"Simple Telescope Simulator";
}

void SG1::SlewTo(double Ra,double Dec){

    Vector2 pos;
    pos.x = Ra;
    pos.y = Dec;

    SendCommand command;
    command.action = ActionType::GoTo;
    command.bufLen = sizeof(Vector2);
    command.buf = (uint8_t*)&pos;

    auto commandBaseLen = sizeof(ActionType)+sizeof(uint8_t);
    char byteData[command.bufLen+commandBaseLen];
    memcpy(byteData,&command,commandBaseLen);
    memcpy(byteData+commandBaseLen,command.buf,command.bufLen);

    writeToSerialPort(SerialPort,byteData,commandBaseLen+command.bufLen);
}

bool SG1::Goto(double ra, double dec)
{
    DEBUGF(DBG_SG1, "Goto - Celestial reference frame target right ascension %lf(%lf) declination %lf",
           ra * 360.0 / 24.0, ra, dec);

    // Call the alignment subsystem to translate the celestial reference frame coordinate
    // into a telescope reference frame coordinate
    TelescopeDirectionVector TDV;
    INDI::IHorizontalCoordinates AltAz { 0, 0 };

    if (TransformCelestialToTelescope(ra, dec, 0.0, TDV))
    {
        // The alignment subsystem has successfully transformed my coordinate
        AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
    }
    else
    {
        // The alignment subsystem cannot transform the coordinate.
        // Try some simple rotations using the stored observatory position if any

        INDI::IEquatorialCoordinates EquatorialCoordinates { ra, dec };
        INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &AltAz);
        TDV = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
        switch (GetApproximateMountAlignment())
        {
            case ZENITH:
                break;

            case NORTH_CELESTIAL_POLE:
                // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 minus
                // the (positive)observatory latitude. The vector itself is rotated anticlockwise
                TDV.RotateAroundY(m_Location.latitude - 90.0);
                break;

            case SOUTH_CELESTIAL_POLE:
                // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 plus
                // the (negative)observatory latitude. The vector itself is rotated clockwise
                TDV.RotateAroundY(m_Location.latitude + 90.0);
                break;
        }
        AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
    }

    // My altitude encoder runs -90 to +90
    if ((AltAz.altitude > 90.0) || (AltAz.altitude < -90.0))
    {
        DEBUG(DBG_SG1, "Goto - Altitude out of range");
        // This should not happen
        return false;
    }

    // My polar encoder runs 0 to +360
    if ((AltAz.azimuth > 360.0) || (AltAz.azimuth < -360.0))
    {
        DEBUG(DBG_SG1, "Goto - Azimuth out of range");
        // This should not happen
        return false;
    }

    if (AltAz.azimuth < 0.0)
    {
        DEBUG(DBG_SG1, "Goto - Azimuth negative");
        AltAz.azimuth = 360.0 + AltAz.azimuth;
    }

    DEBUGF(DBG_SG1, "Goto - Scope reference frame target altitude %lf azimuth %lf", AltAz.altitude,
           AltAz.azimuth);

    
    SlewTo(AltAz.azimuth,AltAz.altitude);

    return true;
}

bool SG1::initProperties()
{
    /* Make sure to init parent properties first */
    INDI::Telescope::initProperties();

    TrackState = SCOPE_IDLE;

    /* Add debug controls so we may debug driver if necessary */
    addDebugControl();


    // Add alignment properties
    InitAlignmentProperties(this);

    return true;
}

bool SG1::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                         char *formats[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentBLOBProperties(this, name, sizes, blobsizes, blobs, formats, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

bool SG1::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentNumberProperties(this, name, values, names, n);
    }

    //  if we didn't process it, continue up the chain, let somebody else
    //  give it a shot
    return INDI::Telescope::ISNewNumber(dev, name, values, names, n);
}

bool SG1::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentSwitchProperties(this, name, states, names, n);
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev, name, states, names, n);
}

bool SG1::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Process alignment properties
        ProcessAlignmentTextProperties(this, name, texts, names, n);
    }
    // Pass it up the chain
    return INDI::Telescope::ISNewText(dev, name, texts, names, n);
}

bool SG1::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    double axisDir = (dir == DIRECTION_NORTH) ? 1 : -1;

    if(command == MOTION_START){
        RaConstRate += axisDir;
        RaSlewNorth = dir == DIRECTION_NORTH;
        RaSlewing = true;
    }
    else{
        RaConstRate += (dir == DIRECTION_NORTH) ? -1 : 1;
        RaSlewing = false;
    }

    SetConstRates(RaConstRate,DecConstRate);

    return true;
}

bool SG1::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    double axisDir = (dir == DIRECTION_WEST) ? 1 : -1;

    if(command == MOTION_START){
        DecConstRate += axisDir;
        DecSlewWest = dir == DIRECTION_WEST;
        DecSlewing = true;
    }
    else{
        DecConstRate += DecSlewWest ? -1 : 1;
        DecSlewing = false;
    }

    SetConstRates(RaConstRate,DecConstRate);

    return true;
}

void SG1::QueryAxisPos(){
    SendCommand command;
    command.action = ActionType::SetConstantRate;
    command.bufLen = 0;
    command.buf = nullptr;

    auto commandBaseLen = sizeof(ActionType)+sizeof(uint8_t);
    char byteData[commandBaseLen];
    memcpy(byteData,&command,commandBaseLen);

    writeToSerialPort(SerialPort,byteData,commandBaseLen+command.bufLen);

    char recievedData[sizeof(QueryReport)];
    readFromSerialPort(SerialPort,recievedData,sizeof(QueryReport));
}

bool SG1::ReadScopeStatus()
{
    QueryAxisPos();
    INDI::IHorizontalCoordinates AltAz { CurrentRaAngle, CurrentDecAngle };
    TelescopeDirectionVector TDV = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
    double RightAscension, Declination;
    if (!TransformTelescopeToCelestial(TDV, RightAscension, Declination))
    {
        if (TraceThisTick)
            DEBUG(DBG_SG1, "ReadScopeStatus - TransformTelescopeToCelestial failed");

        TelescopeDirectionVector RotatedTDV(TDV);

        switch (GetApproximateMountAlignment())
        {
            case ZENITH:
                if (TraceThisTick)
                    DEBUG(DBG_SG1, "ReadScopeStatus - ApproximateMountAlignment ZENITH");
                break;

            case NORTH_CELESTIAL_POLE:
                if (TraceThisTick)
                    DEBUG(DBG_SG1, "ReadScopeStatus - ApproximateMountAlignment NORTH_CELESTIAL_POLE");
                // Rotate the TDV coordinate system anticlockwise (positive) around the y axis by 90 minus
                // the (positive)observatory latitude. The vector itself is rotated clockwise
                RotatedTDV.RotateAroundY(90.0 - m_Location.latitude);
                AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, AltAz);
                break;

            case SOUTH_CELESTIAL_POLE:
                if (TraceThisTick)
                    DEBUG(DBG_SG1, "ReadScopeStatus - ApproximateMountAlignment SOUTH_CELESTIAL_POLE");
                // Rotate the TDV coordinate system clockwise (negative) around the y axis by 90 plus
                // the (negative)observatory latitude. The vector itself is rotated anticlockwise
                RotatedTDV.RotateAroundY(-90.0 - m_Location.latitude);
                AltitudeAzimuthFromTelescopeDirectionVector(RotatedTDV, AltAz);
                break;
        }

        INDI::IEquatorialCoordinates EquatorialCoordinates;
        INDI::HorizontalToEquatorial(&AltAz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);
        // libnova works in decimal degrees
        RightAscension = EquatorialCoordinates.rightascension;
        Declination    = EquatorialCoordinates.declination;
    }

    if (TraceThisTick)
        DEBUGF(DBG_SG1, "ReadScopeStatus - RA %lf hours DEC %lf degrees", RightAscension, Declination);

    NewRaDec(RightAscension, Declination);

    return true;
}

bool SG1::Sync(double ra, double dec)
{
    QueryAxisPos();
    INDI::IHorizontalCoordinates AltAz { 0, 0 };
    AlignmentDatabaseEntry NewEntry;

    AltAz.altitude = CurrentDecAngle;
    AltAz.azimuth  = CurrentRaAngle;

    NewEntry.ObservationJulianDate = ln_get_julian_from_sys();
    NewEntry.RightAscension        = ra;
    NewEntry.Declination           = dec;
    NewEntry.TelescopeDirection    = TelescopeDirectionVectorFromAltitudeAzimuth(AltAz);
    NewEntry.PrivateDataSize       = 0;

    if (!CheckForDuplicateSyncPoint(NewEntry))
    {
        GetAlignmentDatabase().push_back(NewEntry);

        // Tell the client about size change
        UpdateSize();

        // Tell the math plugin to reinitialise
        Initialise(this);

        return true;
    }
    return false;
}

void SG1::SetConstRates(double RaRate, double DecRate){
    Vector2 pos;
    pos.x = RaRate;
    pos.y = DecRate;

    SendCommand command;
    command.action = ActionType::SetConstantRate;
    command.bufLen = sizeof(Vector2);
    command.buf = (uint8_t*)&pos;

    auto commandBaseLen = sizeof(ActionType)+sizeof(uint8_t);
    char byteData[command.bufLen+commandBaseLen];
    memcpy(byteData,&command,commandBaseLen);
    memcpy(byteData+commandBaseLen,command.buf,command.bufLen);

    writeToSerialPort(SerialPort,byteData,commandBaseLen+command.bufLen);
}

void SG1::TimerHit()
{
    TraceThisTickCount++;
    if (60 == TraceThisTickCount)
    {
        TraceThisTick      = true;
        TraceThisTickCount = 0;
    }

    INDI::Telescope::TimerHit(); // This will call ReadScopeStatus

    // OK I have updated the celestial reference frame RA/DEC in ReadScopeStatus
    // Now handle the tracking state
    switch (TrackState)
    {
        case SCOPE_SLEWING:
            if ((STOPPED == AxisStatusRA) && (STOPPED == AxisStatusDEC))
            {
                if (CoordSP.isSwitchOn("TRACK"))
                {
                    // Goto has finished start tracking
                    DEBUG(DBG_SG1, "TimerHit - Goto finished start tracking");
                    TrackState = SCOPE_TRACKING;
                    // Fall through to tracking case
                }
                else
                {
                    TrackState = SCOPE_IDLE;
                    break;
                }
            }
            break;

        case SCOPE_TRACKING:
        {
            double JulianOffset = 1.0 / (24.0 * 60 * 60);
            TelescopeDirectionVector TDV;
            
            INDI::IHorizontalCoordinates AltAz { 0, 0 };

            if (TransformCelestialToTelescope(CurrentTrackingTarget.rightascension, CurrentTrackingTarget.declination, JulianOffset,
                                              TDV))
                AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);
            else
            {

                INDI::IEquatorialCoordinates EquatorialCoordinates { 0, 0 };
                EquatorialCoordinates.rightascension  = CurrentTrackingTarget.rightascension;
                EquatorialCoordinates.declination = CurrentTrackingTarget.declination;
                INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys() + JulianOffset, &AltAz);
                INDI::EquatorialToHorizontal(&EquatorialCoordinates, &m_Location, ln_get_julian_from_sys() + JulianOffset,
                                             &AltAz);

            }

            // My altitude encoder runs -90 to +90
            if ((AltAz.altitude > 90.0) || (AltAz.altitude < -90.0))
            {
                DEBUG(DBG_SG1, "TimerHit tracking - Altitude out of range");
                // This should not happen
                return;
            }

            // My polar encoder runs 0 to +360
            if ((AltAz.azimuth > 360.0) || (AltAz.azimuth < -360.0))
            {
                DEBUG(DBG_SG1, "TimerHit tracking - Azimuth out of range");
                // This should not happen
                return;
            }

            if (AltAz.azimuth < 0.0)
            {
                DEBUG(DBG_SG1, "TimerHit tracking - Azimuth negative");
                AltAz.azimuth = 360.0 + AltAz.azimuth;
            }

            double RaSlew = RaSlewing ? (RaSlewNorth ? 1 : -1) : 0;
            double DecSlew = DecSlewing ? (DecSlewWest ? 1 : -1) : 0;

            SetConstRates(AltAz.azimuth-CurrentRaAngle+RaSlew,AltAz.altitude-CurrentDecAngle+DecSlew);

            break;
        }

        default:
            break;
    }

    TraceThisTick = false;
}

bool SG1::updateLocation(double latitude, double longitude, double elevation)
{
    UpdateLocation(latitude, longitude, elevation);
    return true;
}
