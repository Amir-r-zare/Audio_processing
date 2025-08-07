#include <iostream>
#include <QTcpServer>
#include <QTcpSocket>

using namespace std;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: server <bind_ip> \n";
        return 1;
    }
    QString bindIp = argv[1];
    QTcpServer server;
    if (!server.listen(QHostAddress(bindIp), 1234)) {
        cerr << "Server error: " << server.errorString().toStdString() <<"\n";
        return 1;
    }
    cout << "Server listening on " << bindIp.toStdString() << ":1234" << "\n";
    QObject::connect(&server, &QTcpServer::newConnection, [&]() {
        QTcpSocket *clientSocket = server.nextPendingConnection();
        QObject::connect(clientSocket, &QTcpSocket::readyRead, [clientSocket]() {
            std::cout << "Received: " << clientSocket->readAll().toStdString() << std::endl;
        });
    });

    return 0;
}

