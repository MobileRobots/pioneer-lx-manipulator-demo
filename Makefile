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

ARIA_INCLUDE:=-I$(ARIA)/include -I$(ARIA)/ArNetworking/include -I$(ARIA)/ArVideo/include
ARIA_LINK:=-L$(ARIA)/lib -lArVideo -lArNetworking -lAria -ldl -lpthread -lrt

all: demo Example_CartesianControl

clean: 
	-rm demo

demo: demo.cc ArnlASyncTask.h
	$(CXX) -fPIC -g -o $@ -I$(KINOVA_INCLUDE_DIR) $(ARIA_INCLUDE) $< $(KINOVA_LINK) $(ARIA_LINK)

Example_CartesianControl: Example_CartesianControl.cpp
	$(CXX) -fPIC -g -o $@ -I$(KINOVA_INCLUDE_DIR) $< $(KINOVA_LINK) -ldl


.PHONY: all clean

