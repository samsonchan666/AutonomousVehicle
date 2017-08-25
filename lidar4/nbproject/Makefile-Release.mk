#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux
CND_DLIB_EXT=so
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/include/IMU/ADXL345.o \
	${OBJECTDIR}/include/IMU/HMC5883L.o \
	${OBJECTDIR}/include/IMU/I2Cdev.o \
	${OBJECTDIR}/include/IMU/ITG3200.o \
	${OBJECTDIR}/include/motor.o \
	${OBJECTDIR}/include/ultrasound.o \
	${OBJECTDIR}/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lidar4

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lidar4: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/lidar4 ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/include/IMU/ADXL345.o: include/IMU/ADXL345.cpp
	${MKDIR} -p ${OBJECTDIR}/include/IMU
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/IMU/ADXL345.o include/IMU/ADXL345.cpp

${OBJECTDIR}/include/IMU/HMC5883L.o: include/IMU/HMC5883L.cpp
	${MKDIR} -p ${OBJECTDIR}/include/IMU
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/IMU/HMC5883L.o include/IMU/HMC5883L.cpp

${OBJECTDIR}/include/IMU/I2Cdev.o: include/IMU/I2Cdev.cpp
	${MKDIR} -p ${OBJECTDIR}/include/IMU
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/IMU/I2Cdev.o include/IMU/I2Cdev.cpp

${OBJECTDIR}/include/IMU/ITG3200.o: include/IMU/ITG3200.cpp
	${MKDIR} -p ${OBJECTDIR}/include/IMU
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/IMU/ITG3200.o include/IMU/ITG3200.cpp

${OBJECTDIR}/include/motor.o: include/motor.cpp
	${MKDIR} -p ${OBJECTDIR}/include
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/motor.o include/motor.cpp

${OBJECTDIR}/include/ultrasound.o: include/ultrasound.cpp
	${MKDIR} -p ${OBJECTDIR}/include
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/include/ultrasound.o include/ultrasound.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
