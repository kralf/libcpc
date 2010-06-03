cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libcpc
  VERSION 2.42
  RELEASE stable
  SUMMARY "CPC interface library by Dr. Thomas Wuensche."
  AUTHOR "Dr. Thomas Wuensche"
  CONTACT "sales@ems-wuensche.com"
  HOME "http://www.ems-wuensche.de"
  LICENSE "Propietary License"
  INSTALL /usr
)

remake_pack_deb()
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  libcpc
)
