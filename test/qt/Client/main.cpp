#include <iostream>
#include <QTcpSocket>

using namespace std;

int main(int argc, char *argv[]) {
    if (argc < 2) {
        cerr << "Usage: client <server_ip> \n";
        return 1;
    }
    QString serverIp = argv[1];
    QTcpSocket socket;
    socket.connectToHost(serverIp, 1234);

    if (!socket.waitForConnected(3000)) {
        cerr << "Connection failed: " << socket.errorString().toStdString() << "\n";
        return 1;
    }
    socket.write("Hello. this is client...");
    socket.waitForBytesWritten();
    socket.disconnectFromHost();

    return 0;
}

