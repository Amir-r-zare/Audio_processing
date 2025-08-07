#include <QCoreApplication>
#include <QTcpServer>
#include <QTcpSocket>
#include <iostream>
#include "audio_processing.h"

class SimpleServer : public QObject {
//    Q_OBJECT
public:
    SimpleServer(QObject* parent = nullptr) : QObject(parent) {
        connect(&server, &QTcpServer::newConnection, this, &SimpleServer::onNewConnection);

        if (!server.listen(QHostAddress::Any, 12345)) {
            std::cerr << "Server could not start!" << std::endl;
        } else {
            std::cout << "Server listening on port 12345..." << std::endl;
        }
    }

private slots:
    void onNewConnection() {
        QTcpSocket* client = server.nextPendingConnection();
        std::cout << "Client connected!" << std::endl;

        AudioProcessing aec_qt;
        aec_qt.setConfig(AudioProcessing::SAMPLE_RATE, 48000);
        aec_qt.start();
        std::cout << "[Server] audio_processing started." << std::endl;

        QByteArray welcome = "Welcome from Qt server!\n";
        client->write(welcome);
        client->flush();
        client->waitForBytesWritten();
        client->close();
    }

private:
    QTcpServer server;
};


int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    SimpleServer server;
    return a.exec();
}


// for test, open erminal and write "nc 127.0.0.1 12345"
