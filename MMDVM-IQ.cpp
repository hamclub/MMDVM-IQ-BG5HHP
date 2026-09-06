/*
 *   Copyright (C) 2015,2016,2017,2018,2020,2021,2025,2026 by Jonathan Naylor G4KLX
 *   Copyright (C) 2016 by Mathis Schmieder DB9MAT
 *   Copyright (C) 2016 by Colin Durbridge G4EML
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "MMDVM-IQ.h"
#include "Modem.h"
#include "SDRMulti.h"
#include "SDRSoapy.h"
#if defined(USE_SOAPY_MULTI)
#include "SDRSoapyMulti.h"
#endif
#include "Config.h"
#include "Globals.h"
#include "Version.h"
#include "Thread.h"
#include "Conf.h"
#include "Log.h"
#include "GitVersion.h"

#if !defined(_WIN32) && !defined(_WIN64)
#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <pwd.h>
#endif

#if defined(_WIN32) || defined(_WIN64)
const char* DEFAULT_INI_FILE = "MMDVM-IQ.ini";
#else
const char* DEFAULT_INI_FILE = "/etc/MMDVM-IQ.ini";
#endif

static bool m_killed = false;
static int  m_signal = 0;
static bool m_reload = false;

static const uint8_t  MAX_MMDVM_MODEMS = 4;     // max 4 mmdvm-iq modems currently

#if !defined(_WIN32) && !defined(_WIN64)
static void sigHandler1(int signum)
{
    m_killed = true;
    m_signal = signum;
}

static void sigHandler2(int signum)
{
    m_reload = true;
}
#endif

int main(int argc, char** argv)
{
    const char* iniFile = DEFAULT_INI_FILE;
    if (argc > 1) {
        for (int currentArg = 1; currentArg < argc; ++currentArg) {
            std::string arg = argv[currentArg];
            if ((arg == "-v") || (arg == "--version")) {
                ::fprintf(stdout, "MMDVM-IQ version %s git #%.7s\n", VERSION, gitversion);
                return 0;
            } else if (arg.substr(0, 1) == "-") {
                ::fprintf(stderr, "Usage: MMDVM-IQ [-v|--version] [filename]\n");
                return 1;
            } else {
                iniFile = argv[currentArg];
            }
        }
    }

#if !defined(_WIN32) && !defined(_WIN64)
    ::signal(SIGINT, sigHandler1);
    ::signal(SIGTERM, sigHandler1);
    ::signal(SIGHUP, sigHandler1);
    ::signal(SIGUSR1, sigHandler2);
#endif

    int ret = 0;

    do {
        m_signal = 0;

        CMMDVMIQ* mmdvm = new CMMDVMIQ(std::string(iniFile));
        ret = mmdvm->run();
        delete mmdvm;

        switch (m_signal) {
        case 2:
            ::LogInfo("MMDVM-IQ-%s exited on receipt of SIGINT", VERSION);
            break;
        case 15:
            ::LogInfo("MMDVM-IQ-%s exited on receipt of SIGTERM", VERSION);
            break;
        case 1:
            ::LogInfo("MMDVM-IQ-%s exited on receipt of SIGHUP", VERSION);
            break;
        case 10:
            ::LogInfo("MMDVM-IQ-%s is restarting on receipt of SIGUSR1", VERSION);
            break;
        default:
            ::LogInfo("MMDVM-IQ-%s exited on receipt of an unknown signal", VERSION);
            break;
        }
    } while (m_signal == 10);

    ::LogFinalise();

    return ret;
}

CMMDVMIQ::CMMDVMIQ(const std::string& filename) :
m_filename(filename)
{
}

CMMDVMIQ::~CMMDVMIQ()
{
}

int CMMDVMIQ::run()
{
    CConf conf(m_filename);

    bool ret = conf.read();
    if (!ret) {
        ::fprintf(stderr, "MMDVM-IQ: cannot read the .ini file\n");
        return 1;
    }

#if !defined(_WIN32) && !defined(_WIN64)
    bool m_daemon = conf.getDaemon();
    if (m_daemon) {
        // Create new process
        pid_t pid = ::fork();
        if (pid == -1) {
            ::fprintf(stderr, "Couldn't fork() , exiting\n");
            return -1;
        } else if (pid != 0) {
            exit(EXIT_SUCCESS);
        }

        // Create new session and process group
        if (::setsid() == -1) {
            ::fprintf(stderr, "Couldn't setsid(), exiting\n");
            return -1;
        }

        // Set the working directory to the root directory
        if (::chdir("/") == -1) {
            ::fprintf(stderr, "Couldn't cd /, exiting\n");
            return -1;
        }

        // If we are currently root...
        if (getuid() == 0) {
            struct passwd* user = ::getpwnam("mmdvm");
            if (user == nullptr) {
                ::fprintf(stderr, "Could not get the mmdvm user, exiting\n");
                return -1;
            }

            uid_t mmdvm_uid = user->pw_uid;
            gid_t mmdvm_gid = user->pw_gid;

            // Set user and group ID's to mmdvm:mmdvm
            if (::setgid(mmdvm_gid) != 0) {
                ::fprintf(stderr, "Could not set mmdvm GID, exiting\n");
                return -1;
            }

            if (::setuid(mmdvm_uid) != 0) {
                ::fprintf(stderr, "Could not set mmdvm UID, exiting\n");
                return -1;
            }

            // Double check it worked (AKA Paranoia)
            if (::setuid(0) != -1) {
                ::fprintf(stderr, "It's possible to regain root - something is wrong!, exiting\n");
                return -1;
            }
        }
    }
#endif
#if defined(USE_MQTT) && USE_MQTT == 1
    ::LogInitialise(conf.getLogDisplayLevel(), conf.getLogMQTTLevel());

    std::vector<std::pair<std::string, void (*)(const unsigned char*, unsigned int)>> subscriptions;
    m_mqtt = new CMQTTConnection(conf.getMQTTHost(), conf.getMQTTPort(), conf.getMQTTName(), conf.getMQTTAuthEnabled(), conf.getMQTTUsername(), conf.getMQTTPassword(), subscriptions, conf.getMQTTKeepalive());
    ret = m_mqtt->open();
    if (!ret) {
        ::fprintf(stderr, "MMDVM-IQ: unable to start the MQTT Publisher\n");
        delete m_mqtt;
        m_mqtt = nullptr;
    }
#else
    bool logUTC = false;
    ::LogInitialiseFile(conf.getDaemon(), conf.getLogFilePath().c_str(), conf.getLogFileRoot().c_str(), conf.getLogFileLevel(), conf.getLogDisplayLevel(), logUTC);
#endif

    uint8_t ver = conf.getModemVersion();
    LogMessage("Modem version: %u", ver);

    // Create the SDR device singleton
    unsigned int activeModems = 1;

    ISDRDevice *sdrDevice = nullptr;

    std::string driver = conf.getModemDriver();

    if (driver == "Multi") {
        LogDebug("MultiModem network mode enabled");
        activeModems = 1;
        sdrDevice = new CSDRMulti(&conf);
    } else {
#if defined(USE_SOAPY_MULTI)
        activeModems = conf.getActiveChannels();
        if (activeModems > MAX_MMDVM_MODEMS)
            activeModems = MAX_MMDVM_MODEMS;

        if (activeModems < 1)
            activeModems = 1;

        if (!conf.getDisableMulti())
            sdrDevice = new CSDRSoapyMulti(&conf);
        else {
            // Force to run single channel mode
            LogWarning("SDRSoapyMulti is disabled, force 1 channel mode");
            activeModems = 1;
            sdrDevice = new CSDRSoapy(&conf);     // fall back to the single threaded implementation
        }

#elif defined(USE_SOAPY)
        activeModems = 1;
        sdrDevice = new CSDRSoapy(&conf);

#else
        ::LogFatal("The SoapySDR interface isn't supported in this build");
        return 1;
#endif
    }

    if (activeModems > 1)
        LogInfo("Multi Channels/Modems: %u", activeModems);

    // Create multi modem instances
    CIO* modems[MAX_MMDVM_MODEMS];
    ::memset(modems, 0, sizeof(modems));

    for (unsigned int i = 0; i < activeModems; i++) {
        CIO* io = new CIO(i);

        // setup sdr device
        sdrDevice->setIO(io, i);
        io->setSDRDevice(sdrDevice);

        // start io(serial,sdr)
        ret = io->start(&conf);
        if (!ret) {
            LogError("Unable to open the modem");
            return 1;
        }

        modems[i] = io;
    }

    LogInfo("MMDVM-IQ-%s is starting", VERSION);
    LogInfo("Built %s %s (GitID #%.7s)", __TIME__, __DATE__, gitversion);

    while (!m_killed) {

      for (unsigned int i = 0; i < activeModems; i ++) {
        modems[i]->process();
      }

    //   if (driver == "Multi")
        CThread::sleep(1U);
    }

    LogInfo("MMDVM-IQ is stopping");

    for (unsigned int i = 0; i < activeModems; i++) {
        CIO* io = modems[i];
        io->stop();
    }

    CThread::sleep(250U);

    for (unsigned int i = 0; i < activeModems; i++) {
        CIO* io = modems[i];
        delete io;
        modems[i] = nullptr;
    }

    delete sdrDevice;

    return 0;
}
