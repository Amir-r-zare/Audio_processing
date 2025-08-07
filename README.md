# Qt Client/Server Test

This project demonstrates a simple TCP client-server architecture using **Qt5** (`QtNetwork` module).

The test is divided into two parts:

- **Server** (`test/qt/server`)
- **Client** (`test/qt/client`)

Both the client and server use Qt's `QTcpSocket` and `QTcpServer` for communication. 
The IP address and port must be passed via command line arguments at runtime.


Folder Structure :

test/
└── qt/
├── client/
│ ├── main.cpp
│ └── CMakeLists.txt
│
├── server/
│ ├── main.cpp
│ └── CMakeLists.txt
│
├── CMakeLists.txt
└── README.md



## Build Instructions

1. Open a terminal in the **root of your project** (where the top-level `CMakeLists.txt` is located).

2. Create a build directory and run CMake:


```bash
mkdir -p build
cd build
cmake ..
make

---

If everything is set up correctly, this will build both:

./test/qt/client/QtClient
./test/qt/server/QtServer


How to Run :
    - Start the Server:
    Run the server by specifying an IP address and port number:
    "./test/qt/server/QtServer <IP> <PORT>"
    
    - Start the Client:
    Run the client with the same IP and port to connect to the server:
    "./test/qt/client/QtClient <IP> <PORT>"




