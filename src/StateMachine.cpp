//
//
//
//
//

#include "StateMachine.h"

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>

#include <std_msgs/String.h>

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::init(int _argc, char**_argv) {
    
    LogManager::init("PubLeica_" + std::to_string(time(NULL)));
    mLeicaListeningThread = std::thread(&StateMachine::leicaListenCallback, this);
 
    ros::NodeHandle n;
    mPubFinalPosition = n.advertise<std_msgs::Float32MultiArray>("/leica/end_position", 0);
    
    std::cout << "Started publisher Leica" << std::endl;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::run() {
 
    std::cout << "Starting main loop" << std::endl;
    while (ros::ok()) {

	bool poseEstimated = false;
        poseEstimated = leicaPosEstimate();

        if (poseEstimated) {
            std_msgs::Float32MultiArray finalPos;
            finalPos.data = std::vector<float>({mCurrentX, mCurrentY, mCurrentZ});
            mPubFinalPosition.publish(finalPos);    
        }
	std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

//---------------------------------------------------------------------------------------------------------------------
bool StateMachine::leicaPosEstimate() {
        mSecureLeica.lock();
        mCurrentX = mLastLeicaX;
        mCurrentY = mLastLeicaY;
        mCurrentZ = mLastLeicaZ;
        mSecureLeica.unlock();

        return true;
}

//---------------------------------------------------------------------------------------------------------------------
int StateMachine::initSocket(int _port) {
    int conexion_servidor, conexion_cliente; //declaramos las variables
    socklen_t longc;                         //Debemos declarar una variable que contendrá la longitud de la estructura
    struct sockaddr_in servidor, cliente;

    conexion_servidor = socket(AF_INET, SOCK_STREAM, 0); //creamos el socket
    // bzero((char *)&servidor, sizeof(servidor)); //llenamos la estructura de 0's
    servidor.sin_family = AF_INET; //asignamos a la estructura
    servidor.sin_port = htons(_port);
    servidor.sin_addr.s_addr = INADDR_ANY; //esta macro especifica nuestra dirección
    if (bind(conexion_servidor, (struct sockaddr *)&servidor, sizeof(servidor)) < 0) { //asignamos un puerto al socket
        LogManager::get()->error("Error reserving socket");
        close(conexion_servidor);
        return 1;
    }
    listen(conexion_servidor, 1); //Estamos a la escucha
    LogManager::get()->status("Listening on port " + std::to_string(ntohs(servidor.sin_port)) + "\n", true);
    longc = sizeof(cliente);                                                           //Asignamos el tamaño de la estructura a esta variable
    conexion_cliente = accept(conexion_servidor, (struct sockaddr *)&cliente, &longc); //Esperamos una conexion
    if (conexion_cliente < 0) {
        LogManager::get()->error("Error stablishing the connection");
        close(conexion_servidor);
        return 1;
    }

    LogManager::get()->status("Connected to " + std::string(inet_ntoa(cliente.sin_addr)) + ":" + std::to_string(htons(cliente.sin_port)), true);
    return conexion_cliente;
}

//---------------------------------------------------------------------------------------------------------------------
int StateMachine::split(std::string message, int len, std::vector<std::vector<std::string>> &medidas) {
    int pos = 0, length = 0;
    int cont = 0;

    int idx0 = message.find("{");
    if (idx0 == -1) {
        return -1;
    }

    int idxEnd = message.find("}");

    std::string dataStr = message.substr(idx0 + 1, idxEnd);
    //std::cout << dataStr << std::endl;
    medidas.resize(1);
    medidas[0].push_back(dataStr.substr(0, dataStr.find(";")));
    dataStr = dataStr.substr(dataStr.find(";") + 1);
    medidas[0].push_back(dataStr.substr(0, dataStr.find(";")));
    dataStr = dataStr.substr(dataStr.find(";") + 1);
    medidas[0].push_back(dataStr);

    return 1;
}

void StateMachine::leicaListenCallback(){
    int socketConnection = initSocket(8000); //666 CHECK IF INPUT PARAM IF NOT DO NOT INIT THREAD
    char buffer[1000];
    while (ros::ok() && socketConnection != -1) {
        int len = recv(socketConnection, buffer, 1000, 0);
        if (len > 0) {
            LogManager::get()->status("Received " + std::to_string(len) + " bytes.");
            std::vector<std::vector<std::string>> receivedParams;
            int numMeasures = split(buffer, len, receivedParams);

            if (receivedParams.size() != 0) {
                try {
                    mSecureLeica.lock();
                    mLastLeicaX = stof(receivedParams.back()[0]); // 666 CHECK DISTANCE PRISMA TO PIXHAWK
                    mLastLeicaY = stof(receivedParams.back()[1]);
                    mLastLeicaZ = stof(receivedParams.back()[2]);
                    mSecureLeica.unlock();
                }
                catch (const std::exception &e) {
                    mSecureLeica.unlock();
                    std::cout << "Error decoding data" << std::endl;
                }
                LogManager::get()->status("New data: " + std::to_string(mLastLeicaX) + ", " + std::to_string(mLastLeicaY) + ", " + std::to_string(mLastLeicaZ), false);
            }
        }
        else {
            LogManager::get()->error("Error receiving data\n", true);
            close(socketConnection);
            socketConnection = -1;
        }
    }
}
