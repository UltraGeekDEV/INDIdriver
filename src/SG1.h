#pragma once

#include "indiguiderinterface.h"
#include "inditelescope.h"
#include "alignment/AlignmentSubsystemForDrivers.h"

class SG1 : public INDI::Telescope, public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
{
    public:
        SG1();

    protected:
        bool Abort() override;
        bool Connect() override;
        bool Disconnect() override;
        const char *getDefaultName() override;
        bool Goto(double ra, double dec) override;
        bool initProperties() override;
        bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[],
                       char *formats[], char *names[], int n) override;
        bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
        bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
        bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
        bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
        bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
        bool ReadScopeStatus() override;
        bool Sync(double ra, double dec) override;
        void TimerHit() override;
        bool updateLocation(double latitude, double longitude, double elevation) override;

    private:
        void QueryAxisPos();
        void SetConstRates(double RaRate,double DecRate);
        void SlewTo(double RaRate,double DecRate);
        enum AxisStatus
        {
            STOPPED,
            SLEWING,
            SLEWING_TO
        };

        AxisStatus AxisStatusDEC { STOPPED };
        double CurrentDecAngle { 0 };
        double DecConstRate { 0 };
        bool DecSlewWest { true };
        bool DecSlewing { false };

        AxisStatus AxisStatusRA { STOPPED };
        double CurrentRaAngle { 0 };
        double RaConstRate { 0 };
        bool RaSlewNorth { true };
        bool RaSlewing { false };

        // Tracking
        INDI::IEquatorialCoordinates CurrentTrackingTarget { 0, 0 };
        // Tracing in timer tick
        int TraceThisTickCount { 0 };
        bool TraceThisTick { false };

        int SerialPort {-1};

        unsigned int DBG_SG1 { 0 };

        enum class ActionType : uint8_t
        {
            SetConstantRate,
            Sync,
            GoTo,
            ReportAxis,
            Dither,
            Guide
        };

        struct Vector2{
            double x;
            double y;
        };

        struct SendCommand{
            ActionType action;
            uint8_t bufLen;
            uint8_t* buf;
        };

        enum class MountStatus{
            Idle,
            Tracking,
            Slewing
        };

        struct QueryReport{
            MountStatus status;
            double RaAngle;
            double DecAngle;
          };
};