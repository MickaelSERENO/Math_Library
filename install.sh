#!/usr/bin/bash

mkdir android_arm_build;
mkdir android_x86_build;
mkdir linux_x64_build;

ANDROID_NDK=/opt/android-ndk

#Android ARM
cd android_arm_build
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/mickael/.local/android/armeabi-v7a/lib/pkgconfig/ cmake ../ -G Ninja -DCMAKE_SYSTEM_NAME=Android -DCMAKE_ANDROID_ARCH_ABI=armeabi-v7a -DANDROID_ABI=armeabi-v7a -DANDROID_NDK=$ANDROID_NDK -DANDROID_PLATFORM=android-21 -DCMAKE_SYSTEM_VERSION=21 -DCMAKE_INSTALL_PREFIX=/home/mickael/.local/android/armeabi-v7a/ -DCMAKE_CXX_FLAGS="-I /home/mickael/.local/android/common_abi/include" -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake
ninja install
cd ../

#Android X86
cd android_x86_build
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/mickael/.local/android/x86/lib/pkgconfig/ cmake ../ -G Ninja -DCMAKE_SYSTEM_NAME=Android -DCMAKE_ANDROID_ARCH_ABI=x86 -DANDROID_ABI=x86 -DANDROID_NDK=/opt/android-ndk/ -DCMAKE_SYSTEM_VERSION=21 -DANDROID_PLATFORM=android-21 -DCMAKE_INSTALL_PREFIX=/home/mickael/.local/android/x86/ -DCMAKE_CXX_FLAGS="-I /home/mickael/.local/android/common_abi/include" -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK/build/cmake/android.toolchain.cmake
ninja install
cd ../

#Linux x64
cd linux_x64_build
PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/home/mickael/.local/lib/pkgconfig/ cmake ../ -G Ninja -DCMAKE_INSTALL_PREFIX=/home/mickael/.local/
ninja install
cd ../
