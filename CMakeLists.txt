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
  HOME "http://aslforge.ethz.ch/projects/libelrob"
  LICENSE "GNU Lesser General Public License (LGPL)"
  PREFIX elrob-
  INSTALL /usr
)

remake_doc(
  html
)

remake_pack_deb(
  libelrob
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  libelrob
)
remake_pack_deb(
  ARCH all
  COMPONENT doc
  DESCRIPTION "documentation"
)
