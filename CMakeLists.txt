cmake_minimum_required(VERSION 2.6)

include(ReMake)

remake_project(
  libcpc
  2.42
  stable
  "CPC interface library by Dr. Thomas Wuensche."
  "Dr. Thomas Wuensche"
  "sales@ems-wuensche.com"
  "http://www.ems-wuensche.de"
  "Propietary License"
  INSTALL /usr
)

remake_pack_deb()
remake_pack_deb(COMPONENT dev)
