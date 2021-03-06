QT += core \
        gui \
        widgets \
        network \
        opengl

CONFIG += c++14 console
CONFIG -= app_bundle

VERSION  = 1.0.0
DESTDIR  = ../bin
TARGET   = Armorial-Actuator-VSS


# Temporary dirs
OBJECTS_DIR = tmp/obj
MOC_DIR = tmp/moc
UI_DIR = tmp/moc
RCC_DIR = tmp/rc

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
LIBS *= -lprotobuf -lGLU -pthread -lGEARSystem -lomniORB4 -lomnithread -lQt5Core -lpthread

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

system(echo "compiling protobuf" && cd include/3rd_party/protobuf && protoc --cpp_out=../ *.proto && cd ../../..)

SOURCES += \
        exithandler.cpp \
        include/3rd_party/command.pb.cc \
        include/3rd_party/common.pb.cc \
        include/3rd_party/packet.pb.cc \
        include/3rd_party/replacement.pb.cc \
        main.cpp \
        packetmanager/packetmanager.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    include/3rd_party/protobuf/command.proto \
    include/3rd_party/protobuf/common.proto \
    include/3rd_party/protobuf/packet.proto \
    include/3rd_party/protobuf/protobuf.sh \
    include/3rd_party/protobuf/replacement.proto

HEADERS += \
    exithandler.h \
    include/3rd_party/command.pb.h \
    include/3rd_party/common.pb.h \
    include/3rd_party/packet.pb.h \
    include/3rd_party/replacement.pb.h \
    include/timer.h \
    packetmanager/packetmanager.h
