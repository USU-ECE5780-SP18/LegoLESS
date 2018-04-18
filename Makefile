# Change target name
TARGET = skeleton
TARGET_SOURCES := $(TARGET).c
TOPPERS_OSEK_OIL_SOURCE = ./$(TARGET).oil

# nxtOSEK root path
# Modify accordingly
NXTOSEKROOT = ../..

#################################################################
# You should not need to modify below this line
O_PATH ?= build
include $(NXTOSEKROOT)/ecrobot/ecrobot.mak
