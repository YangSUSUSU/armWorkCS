# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;arm_kinematics_solver".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-ltest_planning;-losqp;-lOsqpEigen".split(';') if "-ltest_planning;-losqp;-lOsqpEigen" != "" else []
PROJECT_NAME = "test_planning"
PROJECT_SPACE_DIR = "/home/nikoo/workWS/armWorkCS/install"
PROJECT_VERSION = "0.0.0"
