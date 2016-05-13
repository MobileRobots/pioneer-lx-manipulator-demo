ifdef EMULATOR
KINOVA_INCLUDE_DIR:=../emulate-kinova-api/headers.50200
KINOVA_LINK:=-L../emulate-kinova-api -l:emulate_kinova_50200.so
endif

ifndef KINOVA_INCLUDE_DIR
KINOVA_INCLUDE_DIR:=/usr/include
endif

ifndef KINOVA_LIB_DIR
KINOVA_LIB_DIR:=/usr/lib
endif

ifndef KINOVA_LINK
KINOVA_LINK:=-L$(KINOVA_LIB_DIR) -l:Kinova.API.USBCommandLayerUbuntu.so -l:Kinova.API.CommLayerUbuntu.so
endif

ifndef ARIA
ARIA:=/usr/local/Aria
endif

ifndef FREENECT2_DIR
FREENECT2_DIR=/usr/local
endif

ARIA_INCLUDE:=-I$(ARIA)/include -I$(ARIA)/ArNetworking/include -I$(ARIA)/ArVideo/include
ARIA_LINK:=-L$(ARIA)/lib -lArVideo -lArNetworking -lAria -ljpeg -ldl -lpthread -lrt

FREENECT2_INCLUDE=-I$(FREENECT2_DIR)/include -I$(FREENECT2_DIR)/src/tinythread -I$(FREENECT2_DIR)/include/libfreenect2/ -I$(FREENECT2_DIR)/include/libfreenect2/tinythread
LINK_SPECIAL_LIBUSB=-L$(FREENECT2_DIR)/../../depends/libusb/lib -rdynamic -lusb-1.0 -Wl,-rpath,$(FREENECT2_DIR)/../../depends/libusb/lib:$(FREENECT2_DIR)/lib
OPENCV_LINK=-lopencv_core  -lopencv_imgproc #-lopencv_highgui
FREENECT2_LINK=-L$(FREENECT2_DIR)/lib -lfreenect2 -lturbojpeg -lpthread -lOpenCL $(LINK_SPECIAL_LIBUSB) $(OPENCV_LINK)

all: demo Example_CartesianControl Example_AngularControl

clean: 
	-rm demo
	-rm ArmDemoTask.o
	-rm KinectArVideoServer.o

%.o: %.cpp %.h RemoteArnlTask.h
	$(CXX) -c -fPIC -g -o $@ -I$(KINOVA_INCLUDE_DIR) $(ARIA_INCLUDE) $(FREENECT2_INCLUDE) $<

demo: demo.cc ArmDemoTask.o KinectArVideoServer.o 
	$(CXX) -fPIC -g -o $@ -I$(KINOVA_INCLUDE_DIR) $(ARIA_INCLUDE) $(FREENECT2_INCLUDE) $^ $(KINOVA_LINK) $(ARIA_LINK) $(FREENECT2_LINK)

Example_%: Example_%.cpp
	$(CXX) -fPIC -g -o $@ -I$(KINOVA_INCLUDE_DIR) $< $(KINOVA_LINK) -ldl


.PHONY: all clean

