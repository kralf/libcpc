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

remake_component(
  udev
  CONFIGURATION_DESTINATION /etc/udev
)
remake_add_directories(
  rules
  COMPONENT udev
)

remake_pack_deb(
  COMPONENT kernel-source
  DESCRIPTION "kernel module sources"
  DEPENDS dkms
  EXTRA debian/module/*
)
remake_pack_deb(
  COMPONENT udev
  DESCRIPTION "udev rules"
  DEPENDS libcpc-kernel-source udev
  EXTRA debian/rules/*
)
remake_pack_deb(
  DEPENDS libcpc-udev
)
remake_pack_deb(
  COMPONENT dev
  DESCRIPTION "development headers"
  DEPENDS libcpc
)

remake_distribute_deb(
  DISTRIBUTION lucid
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS dkms remake pkg-config
  PASS CMAKE_BUILD_TYPE LIBCPC_GIT_REVISION
)
remake_distribute_deb(
  DISTRIBUTION precise
  SECTION libs
  UPLOAD ppa:kralf/asl
  DEPENDS dkms remake pkg-config
  PASS CMAKE_BUILD_TYPE LIBCPC_GIT_REVISION
)
