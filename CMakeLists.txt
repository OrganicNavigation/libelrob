cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libelrob
  VERSION 0.1
  RELEASE alpha
  SUMMARY "A C-based utility library used in the 2006 ELROB competition."
  AUTHOR "Pierre Lamon"
  AUTHOR "Sascha Kolski"
  AUTHOR "Dizan Vasquez"
  AUTHOR "Ralf Kaestner"
  CONTACT "ralf.kaestner@gmail.com"
  HOME "http://github.com/kralf/libelrob"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX elrob-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  DEPENDS libgsl0ldbl
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libelrob
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libgsl0-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBELROB_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libgsl0-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBELROB_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION trusty
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS libgsl0-dev remake pkg-config doxygen
  PASS CMAKE_BUILD_TYPE LIBELROB_GIT_REVISION
)
