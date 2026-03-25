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

#include "MMDVM.h"
#include "Config.h"
#include "Globals.h"
#include "Thread.h"
#include "Version.h"
#include "Log.h"
#include "GitVersion.h"

// Global variables
MMDVM_STATE m_modemState = STATE_IDLE;

bool m_dstarEnable  = true;
bool m_dmrEnable    = true;
bool m_ysfEnable    = true;
bool m_p25Enable    = true;
bool m_nxdnEnable   = true;
bool m_pocsagEnable = true;
bool m_fmEnable     = true;

bool m_duplex = true;

bool m_tx  = false;
bool m_dcd = false;

#if defined(MODE_DSTAR)
CDStarRX dstarRX;
CDStarTX dstarTX;
#endif

#if defined(MODE_DMR)
CDMRIdleRX dmrIdleRX;
CDMRRX dmrRX;
CDMRTX dmrTX;

CDMRDMORX dmrDMORX;
CDMRDMOTX dmrDMOTX;
#endif

#if defined(MODE_YSF)
CYSFRX ysfRX;
CYSFTX ysfTX;
#endif

#if defined(MODE_P25)
CP25RX p25RX;
CP25TX p25TX;
#endif

#if defined(MODE_NXDN)
CNXDNRX nxdnRX;
CNXDNTX nxdnTX;
#endif

#if defined(MODE_POCSAG)
CPOCSAGTX  pocsagTX;
#endif

#if defined(MODE_FM)
CFM    fm;
#endif

CCWIdTX cwIdTX;

CModem modem;
CSerialPort serial;
CIO io;

#if defined(_WIN32) || defined(_WIN64)
const char* DEFAULT_INI_FILE = "MMDVM.ini";
#else
const char* DEFAULT_INI_FILE = "/etc/MMDVM.ini";
#endif

static bool m_killed = false;
static int  m_signal = 0;
static bool m_reload = false;

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
                ::fprintf(stdout, "MMDVM-PC version %s git #%.7s\n", VERSION, gitversion);
                return 0;
            } else if (arg.substr(0, 1) == "-") {
                ::fprintf(stderr, "Usage: MMDVM-PC [-v|--version] [filename]\n");
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

        CMMDVM* mmdvm = new CMMDVM(std::string(iniFile));
        ret = mmdvm->run();
        delete mmdvm;

        switch (m_signal) {
        case 2:
            ::LogInfo("MMDVM-PC-%s exited on receipt of SIGINT", VERSION);
            break;
        case 15:
            ::LogInfo("MMDVM-PC-%s exited on receipt of SIGTERM", VERSION);
            break;
        case 1:
            ::LogInfo("MMDVM-PC-%s exited on receipt of SIGHUP", VERSION);
            break;
        case 10:
            ::LogInfo("MMDVM-PC-%s is restarting on receipt of SIGUSR1", VERSION);
            break;
        default:
            ::LogInfo("MMDVM-PC-%s exited on receipt of an unknown signal", VERSION);
            break;
        }
    } while (m_signal == 10);

    ::LogFinalise();

    return ret;
}

CMMDVM::CMMDVM(const std::string& filename) :
m_conf(filename)
{
}

CMMDVM::~CMMDVM()
{
}

int CMMDVM::run()
{
    bool ret = m_conf.read();
    if (!ret) {
        ::fprintf(stderr, "MMDVM-PC: cannot read the .ini file\n");
        return 1;
    }

#if !defined(_WIN32) && !defined(_WIN64)
    bool m_daemon = m_conf.getDaemon();
    if (m_daemon) {
        // Create new process
        pid_t pid = ::fork();
        if (pid == -1) {
            ::fprintf(stderr, "Couldn't fork() , exiting\n");
            return -1;
        }
        else if (pid != 0) {
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
    ::LogInitialise(m_conf.getLogDisplayLevel(), m_conf.getLogMQTTLevel());

    std::vector<std::pair<std::string, void (*)(const unsigned char*, unsigned int)>> subscriptions;
    m_mqtt = new CMQTTConnection(m_conf.getMQTTHost(), m_conf.getMQTTPort(), m_conf.getMQTTName(), m_conf.getMQTTAuthEnabled(), m_conf.getMQTTUsername(), m_conf.getMQTTPassword(), subscriptions, m_conf.getMQTTKeepalive());
    ret = m_mqtt->open();
    if (!ret) {
        ::fprintf(stderr, "MMDVM-PC: unable to start the MQTT Publisher\n");
        delete m_mqtt;
        return 1;
    }

    ret = serial.start(m_conf.getNetworkLocalAddress(), m_conf.getNetworkLocalPort(),
                            m_conf.getNetworkHostAddress(), m_conf.getNetworkHostPort(),
                            m_conf.getNetworkTrace());
    if (!ret) {
        LogError("Unable to open the host network connection");
        return 1;
    }

    ret = modem.start();
    if (!ret) {
        LogError("Unable to open the modem connection");
        return 1;
    }

    LogInfo("MMDVM-PC-%s is starting", VERSION);
    LogInfo("Built %s %s (GitID #%.7s)", __TIME__, __DATE__, gitversion);

    while (!m_killed) {
        CThread::sleep(1U);

        serial.process();

        io.process();

        modem.process();

        // The following is for transmitting
#if defined(MODE_DSTAR)
        if (m_dstarEnable && m_modemState == STATE_DSTAR)
            dstarTX.process();
#endif

#if defined(MODE_DMR)
        if (m_dmrEnable && m_modemState == STATE_DMR) {
            if (m_duplex)
                dmrTX.process();
            else
                dmrDMOTX.process();
        }
#endif

#if defined(MODE_YSF)
        if (m_ysfEnable && m_modemState == STATE_YSF)
            ysfTX.process();
#endif

#if defined(MODE_P25)
        if (m_p25Enable && m_modemState == STATE_P25)
            p25TX.process();
#endif

#if defined(MODE_NXDN)
        if (m_nxdnEnable && m_modemState == STATE_NXDN)
           nxdnTX.process();
#endif

#if defined(MODE_POCSAG)
        if (m_pocsagEnable && (m_modemState == STATE_POCSAG || pocsagTX.busy()))
           pocsagTX.process();
#endif

#if defined(MODE_FM)
        if (m_fmEnable && m_modemState == STATE_FM)
           fm.process();
#endif

        if (m_modemState == STATE_IDLE)
            cwIdTX.process();
    }

    LogInfo("MMDVM-PC is stopping");

    modem.stop();
    serial.stop();

    return 0;
}
