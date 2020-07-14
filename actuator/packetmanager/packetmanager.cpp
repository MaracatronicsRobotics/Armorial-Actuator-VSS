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

#include "packetmanager.h"

PacketManager::PacketManager(const QString &name) : Actuator(name)
{
    for(int x = 0; x < QT_TEAMS; x++){
        for(int y = 0; y < QT_PLAYERS; y++){
            packets[x][y].id = y;
            packets[x][y].isYellow = x ? false : true;
            packets[x][y].wl = 0.0;
            packets[x][y].wr = 0.0;

            packets[x][y].isUpdated = false;
        }
    }

    _running = true;
    _loopTime = 1000;
}

void PacketManager::sendPacket(firaSim_robot robot){
    fira_message::sim_to_ref::Packet packet;
    fira_message::sim_to_ref::Command *command = packet.mutable_cmd()->add_robot_commands();

    command->set_id(robot.id);
    command->set_yellowteam(robot.isYellow);
    command->set_wheel_left(robot.wl);
    command->set_wheel_right(robot.wr);

    std::string s;
    packet.SerializeToString(&s);
    if(_socket.write(s.c_str(), s.length()) == -1)
        std::cout << "[Armorial-Actuator-VSS] Failed to write to socket: " << _socket.errorString().toStdString() << std::endl;
}

void PacketManager::run(){
    Timer loopController;
    while(isRunning()){
        loopController.start();
        if(isConnected()){
            for(int x = 0; x < QT_TEAMS; x++){
                for(int y = 0; y < QT_PLAYERS; y++){
                    _writeMutex.lock();
                    if(packets[x][y].isUpdated){
                        sendPacket(packets[x][y]);
                        markPlayersAsOutdated(x, y);
                    }
                    _writeMutex.unlock();
                }
            }
        }
        loopController.stop();

        // loop time control
        if(isRunning()){
            long rest = _loopTime - loopController.timemsec();
            if(rest >= 0){
                msleep(rest);
            }
            else{
                std::cout << "[TIMER OVEREXTENDED] " << " Armorial-Actuator for " <<  -rest  << " ms.\n";
            }
        }
    }
}

void PacketManager::setLoopFrequency(int hz) {
    if(hz != 0) _loopTime = 1000/hz;
}

void PacketManager::markPlayersAsUpdated(quint8 teamNum, quint8 playerNum) {
    packets[teamNum][playerNum].isUpdated = true;
}

void PacketManager::markPlayersAsOutdated(quint8 teamNum, quint8 playerNum) {
    packets[teamNum][playerNum].isUpdated = false;
}

bool PacketManager::isRunning() {
    bool result;

    _mutexRunning.lock();
    result = _running;
    _mutexRunning.unlock();

    return result;
}

void PacketManager::stopRunning() {
    _mutexRunning.lock();
    _running = false;
    _mutexRunning.unlock();
}

bool PacketManager::connect(const QString& serverAddress, const uint16 serverPort, const QString& firaSimAddress, const quint16 firaSimPort) {
    // Connects to WRBackbone
    if(!Actuator::connect(serverAddress, serverPort)) {
        std::cerr << ">> [Armorial-Actuator] Failed to connect to WRBackbone server!" << std::endl;
        return false;
    }

    // Connects to grSim command listener
    if(_socket.isOpen())
        _socket.close();
    _socket.connectToHost(firaSimAddress, firaSimPort, QIODevice::WriteOnly, QAbstractSocket::IPv4Protocol);

    std::cout << "[Armorial-Actuator] Connected!" << std::endl;

    return true;
}
void PacketManager::disconnect() {
    // Disconnect from WRBackbone
    Actuator::disconnect();

    // Close grSim socket
    if(_socket.isOpen()){
        _socket.close();
    }
}

bool PacketManager::isConnected() const {
    return (_socket.isOpen() && Actuator::isConnected());
}

void PacketManager::setSpeed(quint8 teamNum, quint8 playerNum, float x, float y, float theta) {
    // Save values
    _writeMutex.lock();

    // x = vx desejada
    // y = angular desejada

    // equações pra transformar em vr e vl
    /*
     * Onde:
     * L       = distancia entre as rodas
     * r       = raio da roda
     * VLinear = vx desejada
     * w       = velocidade angular desejada
     *
     * (2 * VLinear)/r = Wr + Wl
     * (w * L)/r       = Wr - Wl
     * Wr              = (2 * Vlin + w * L)/(2 * r)
     * Wl              = Wr - (w * L) / r
    */

    double L = 0.066;
    double r = 0.016;
    double wr = ((2 * x) + (L * y)) / (2 * r);
    double wl = wr - ((L * y) / r);

    packets[teamNum][playerNum].wl    = wl;
    packets[teamNum][playerNum].wr    = wr;
    markPlayersAsUpdated(teamNum, playerNum);
    _writeMutex.unlock();
}

void PacketManager::kick(quint8 teamNum, quint8 playerNum, float power) {

}

void PacketManager::chipKick(quint8 teamNum, quint8 playerNum, float power) {

}

void PacketManager::kickOnTouch(quint8 teamNum, quint8 playerNum, bool enable, float power) {

}

void PacketManager::chipKickOnTouch(quint8 teamNum, quint8 playerNum, bool enable, float power) {

}

void PacketManager::holdBall(quint8 teamNum, quint8 playerNum, bool enable) {

}
