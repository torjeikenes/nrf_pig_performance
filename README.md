# NRF Pig Performance

An activity tracker for pigs consisting of a BLE earchip for each pig, and a sentral that gathers data from each pig over BLE and sends it to the backend over NB-IoT.

Link to code for website: https://github.com/alfhj/pig-performance


## Setup

Download nrf5 SDK v. 12.3.0 into the root folder of the repo
https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download

### Install Arm GNU Toolchain

https://lindevs.com/install-arm-gnu-toolchain-on-ubuntu

I had some problems with 12.2, so I use version 11.3.1 (as of 28/2/23)

`ARM_TOOLCHAIN_VERSION=11.3.Rel1`

`curl -Lo gcc-arm-none-eabi.tar.xz "https://developer.arm.com/-/media/Files/downloads/gnu/${ARM_TOOLCHAIN_VERSION}/binrel/arm-gnu-toolchain-${ARM_TOOLCHAIN_VERSION}-x86_64-arm-none-eabi.tar.xz"`

`sudo mkdir /opt/gcc-arm-none-eabi`

`sudo tar xf gcc-arm-none-eabi.tar.xz --strip-components=1 -C /opt/gcc-arm-none-eabi`

Add `export PATH=$PATH:/opt/gcc-arm-none-eabi/bin` to `.bashrc` or `.zshrc`

Open `<SDK_ROOT>/components/toolchain/gcc/Makefile.posix` and set the correct path

### Install nrf command line tools

https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download

## Build

For nrf51 dk open pca10028/s130/armgcc in the terminal and build using the Makefile.
