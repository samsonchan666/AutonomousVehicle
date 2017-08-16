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
CND_CONF=Debug
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/ADXL345.o \
	${OBJECTDIR}/HMC5883L.o \
	${OBJECTDIR}/I2Cdev.o \
	${OBJECTDIR}/ITG3200.o \
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
LDLIBSOPTIONS=-lwiringPi -lbcm2835 -lpthread

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/test_imu_i2c

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/test_imu_i2c: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/test_imu_i2c ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/ADXL345.o: ADXL345.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -D_REENTRANT\ 1 -include I2Cdev.h -include HMC5883L.h -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ADXL345.o ADXL345.cpp

${OBJECTDIR}/HMC5883L.o: HMC5883L.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -D_REENTRANT\ 1 -include I2Cdev.h -include HMC5883L.h -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/HMC5883L.o HMC5883L.cpp

${OBJECTDIR}/I2Cdev.o: I2Cdev.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -D_REENTRANT\ 1 -include I2Cdev.h -include HMC5883L.h -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/I2Cdev.o I2Cdev.cpp

${OBJECTDIR}/ITG3200.o: ITG3200.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -D_REENTRANT\ 1 -include I2Cdev.h -include HMC5883L.h -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ITG3200.o ITG3200.cpp

${OBJECTDIR}/main.o: main.cpp
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -g -D_REENTRANT\ 1 -include I2Cdev.h -include HMC5883L.h -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

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
