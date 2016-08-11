# Empyt Project Template
# YAO=liang
#生成LIB
# Version required检查CMake版本
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

ADD_LIBRARY(SBG SHARED sbgCom.c comWrapper/comSerialUnix.c protocol/commands.c protocol/commandsCalib.c protocol/commandsFilter.c protocol/commandsNav.c protocol/commandsOrientation.c protocol/protocol.c protocol/protocolOutput.c protocol/protocolOutputMode.c time/sbgTime.c protocol/commandsExt.c protocol/commandsGps.c protocol/commandsOdo.c protocol/commandsOutput.c protocol/commandsSync.c)


