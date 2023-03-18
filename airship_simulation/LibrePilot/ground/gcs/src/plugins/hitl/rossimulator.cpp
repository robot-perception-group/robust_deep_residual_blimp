/**
 ******************************************************************************
 *
 * @file       flightgearbridge.h
 * @author     The LibrePilot Project, http://www.librepilot.org Copyright (C) 2016.
 *             The OpenPilot Team, http://www.openpilot.org Copyright (C) 2015.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup HITLPlugin HITL Plugin
 * @{
 * @brief The Hardware In The Loop plugin
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "rossimulator.h"
#include "extensionsystem/pluginmanager.h"
#include "coreplugin/icore.h"
#include "coreplugin/threadmanager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


ROSSimulator::ROSSimulator(const SimulatorSettings & params) :
    Simulator(params)
{}

ROSSimulator::~ROSSimulator()
{
    disconnect(simProcess, SIGNAL(readyReadStandardOutput()), this, SLOT(processReadyRead()));
}

void ROSSimulator::setupUdpPorts(const QString & host, int inPort, int outPort)
{
    Q_UNUSED(outPort);

    if (inSocket->bind(QHostAddress(host), inPort)) {
        emit processOutput("Successfully bound to address " + host + " on port " + QString::number(inPort) + "\n");
    } else {
        emit processOutput("Cannot bind to address " + host + " on port " + QString::number(inPort) + "\n");
    }
}

bool ROSSimulator::setupProcess()
{
    QMutexLocker locker(&lock);

    Qt::HANDLE mainThread = QThread::currentThreadId();

    qDebug() << "setupProcess Thread: " << mainThread;

    simProcess = new QProcess();
    simProcess->setReadChannelMode(QProcess::MergedChannels);
    connect(simProcess, SIGNAL(readyReadStandardOutput()), this, SLOT(processReadyRead()));
    // Note: Only tested on windows 7
#if defined(Q_WS_WIN)
    QString cmdShell("c:/windows/system32/cmd.exe");
#else
    QString cmdShell("bash");
#endif

    // Start shell (Note: Could not start ROS directly on Windows, only through terminal!)
    simProcess->start(cmdShell);
    if (simProcess->waitForStarted() == false) {
        emit processOutput("Error:" + simProcess->errorString());
        return false;
    }

    // Setup arguments
    // Note: The input generic protocol is set to update at a much higher rate than the actual updates are sent by the GCS.
    // If this is not done then a lag will be introduced by FlightGear, likelly because the receive socket buffer builds up during startup.
    QString args("--connection=udp://" + settings.hostAddress + ":" + QString::number(settings.inPort));

    // Start FlightGear - only if checkbox is selected in HITL options page
    if (settings.startSim) {
        QString cmd("\"" + settings.binPath + "\" " + args + "\n");
        simProcess->write(cmd.toLatin1());
    } else {
        emit processOutput("Start the ROS node python script on the command line, please use the following arguments:\n" +
                           args);
    }

    return true;
}

void ROSSimulator::processReadyRead()
{
    QByteArray bytes = simProcess->readAllStandardOutput();
    QString str(bytes);

    if (!str.contains("Error reading data")) { // ignore error
        emit processOutput(str);
    }
}

void ROSSimulator::transmitUpdate()
{
    ActuatorCommand::DataFields actData = actCommand->getData();

    // Send update to FlightGear
    QString cmd;

    cmd = QString("%1,%2,%3,%4,%5,%6,%7,%8,%9,%10,%11,%12\n")
          .arg(actData.Channel[0])
          .arg(actData.Channel[1])
          .arg(actData.Channel[2])
          .arg(actData.Channel[3])
          .arg(actData.Channel[4])
          .arg(actData.Channel[5])
          .arg(actData.Channel[6])
          .arg(actData.Channel[7])
          .arg(actData.Channel[8])
          .arg(actData.Channel[9])
          .arg(actData.Channel[10])
          .arg(actData.Channel[11]);

    QByteArray data = cmd.toLatin1();

    if (outSocket->writeDatagram(data, QHostAddress(settings.remoteAddress), settings.outPort) == -1) {
        emit processOutput("Error sending UDP packet to ROS: " + outSocket->errorString() + "\n");
    }
}


void ROSSimulator::processUpdate(const QByteArray & inp)
{
    // TODO: this does not use the FLIGHT_PARAM structure, it should!
    // Split
    QString data(inp);
    QStringList fields = data.split(",");

    if (fields.length() < 18) {
        emit processOutput("Received invalid message - too short!\n");
        return;
    }
    // Get rollRate (deg/s)
    float rollRate  = fields[0].toFloat();
    // Get pitchRate (deg/s)
    float pitchRate = fields[1].toFloat();
    // Get yawRate (deg/s)
    float yawRate   = fields[2].toFloat();
    // Get xAccel (m/s^2)
    float xAccel    = fields[3].toFloat();
    // Get yAccel (m/s^2)
    float yAccel    = fields[4].toFloat();
    // Get xAccel (m/s^2)
    float zAccel    = fields[5].toFloat();
    // Get roll (deg)
    float roll     = fields[6].toFloat();
    // Get pitch (deg)
    float pitch    = fields[7].toFloat();
    // Get yaw (deg)
    float yaw      = fields[8].toFloat();
    // Get airspeed (m/s)
    float airspeed = fields[9].toFloat();
    // Get VelocityState Down (m/s)
    float velocityStateNorth = fields[10].toFloat();
    // Get VelocityState East (m/s)
    float velocityStateEast  = fields[11].toFloat();
    // Get VelocityState Down (m/s)
    float velocityStateDown  = fields[12].toFloat();
    // Get PositionState Down (m/s)
    float positionStateNorth = fields[13].toFloat();
    // Get PositionState East (m/s)
    float positionStateEast  = fields[14].toFloat();
    // Get PositionState Down (m/s)
    float positionStateDown  = fields[15].toFloat();
    // Get temperature (degC)
    float temperature = fields[16].toFloat();
    // Get pressure (Pa)
    float pressure    = fields[17].toFloat();

    ///////
    // Output formatting
    ///////
    Output2Hardware out;

    memset(&out, 0, sizeof(Output2Hardware));

    HomeLocation::DataFields homeData = posHome->getData();
    double HomeLLA[3] = { (double)homeData.Latitude * 1e-7, (double)homeData.Longitude * 1e-7, homeData.Altitude };
    double NED[3]     = { positionStateNorth, positionStateEast, positionStateDown };
    double LLA[3];

    Utils::CoordinateConversions().NED2LLA_HomeLLA(HomeLLA, NED, LLA);

    // Update GPS Position objects
    out.latitude     = LLA[0] * 1e7;
    out.longitude    = LLA[1] * 1e7;
    out.altitude     = LLA[2];
    out.agl = LLA[2];
    out.groundspeed  = sqrt(velocityStateNorth * velocityStateNorth + velocityStateEast * velocityStateEast);

    out.calibratedAirspeed = airspeed;
    out.trueAirspeed = airspeed * (1.0 + (0.02 * out.agl / 304.8));


    // Update BaroSensor object
    out.temperature = temperature;
    out.pressure    = pressure;

    // Update attState object
    out.roll      = roll;       // roll;
    out.pitch     = pitch;     // pitch
    out.heading   = yaw; // yaw

    out.dstN      = NED[0];
    out.dstE      = NED[1];
    out.dstD      = NED[2];

    // Update VelocityState.{North,East,Down}
    out.velNorth  = velocityStateNorth;
    out.velEast   = velocityStateEast;
    out.velDown   = velocityStateDown;

    // Update gyroscope sensor data
    out.rollRate  = rollRate;
    out.pitchRate = pitchRate;
    out.yawRate   = yawRate;

    // Update accelerometer sensor data
    out.accX      = xAccel;
    out.accY      = yAccel;
    out.accZ      = -zAccel;

    updateUAVOs(out);
}
