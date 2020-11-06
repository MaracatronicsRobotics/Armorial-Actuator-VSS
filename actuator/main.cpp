/***
 * Maracatronics Robotics
 * Federal University of Pernambuco (UFPE) at Recife
 * http://www.maracatronics.com/
 *
 * This file is part of Armorial project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 ***/

#include <QCoreApplication>
#include <exithandler.h>
#include <packetmanager/packetmanager.h>

/* backbone (localhost?) defines */
#define BACKBONE_ADDRESS "127.0.0.1"
#define BACKBONE_PORT 0

/* grsim host defines */
#define FIRASIM_ADDRESS "127.0.0.1"
#define FIRASIM_PORT 20011

/* thread defines */
#define LOOP_FREQUENCY 60 // hz

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    ExitHandler::setup(&a);

    // Command line parser, get arguments
    QCommandLineParser parser;
    parser.setApplicationDescription("Actuator VSS application help.");
    parser.addHelpOption();
    parser.addVersionOption();
    parser.addPositionalArgument("actuatorAddress", "Sets the address that the application will send info. (default is 127.0.0.1)");
    parser.addPositionalArgument("actuatorPort", "Sets the port that the application will send info. (default is 20011)");
    parser.process(a);
    QStringList args = parser.positionalArguments();

    int firaPort = 20011;
    std::string firaAddress = "127.0.0.1";

    if(args.size() >= 1){
        std::string firaAdd = args.at(0).toStdString();
        firaAddress = firaAdd;
    }

    if(args.size() >= 2){
        int port = args.at(1).toInt();

        if(port < 1 || port > 65535){
            std::cout << "[ERROR] Invalid port: " << port << std::endl;
            return EXIT_FAILURE;
        }

        firaPort = port;
    }

    PacketManager packetManager("Armorial-Actuator-VSS");
    packetManager.connect(BACKBONE_ADDRESS, BACKBONE_PORT, QString(firaAddress.c_str()), static_cast<quint16>(firaPort));
    packetManager.setLoopFrequency(LOOP_FREQUENCY);
    packetManager.start();

    // wait
    int retn = a.exec();

    // finish
    packetManager.stopRunning();
    packetManager.wait();

    return retn;
}
