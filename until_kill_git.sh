#!/bin/bash
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games
export PATH




sudo find ./ | grep "\(hima.*ultra\|b3\)" -iw | sudo xargs rm -rf
sudo rm -rf ./arch/arm64/boot/dts/qcom ; sudo ln -s ../../../arm/boot/dts/qcom/ arch/arm64/boot/dts/
sudo rm -rf ./arch/arm64/boot/dts/include/dt-bindings ; sudo ln -s ../../../../../include/dt-bindings ./arch/arm64/boot/dts/include/dt-bindings
sudo rm -rf ./arch/arm/boot/dts/qcom/skeleton.dtsi ; sudo ln -s ../skeleton.dtsi ./arch/arm/boot/dts/qcom/skeleton.dtsi
sudo rm -rf ./arch/arm/boot/dts/qcom/skeleton64.dtsi ; sudo ln -s ../skeleton64.dtsi ./arch/arm/boot/dts/qcom/skeleton64.dtsi
sudo rm -rf ./arch/arm/boot/dts/include/dt-bindings ; sudo ln -s ../../../../../include/dt-bindings ./arch/arm/boot/dts/include/dt-bindings

sudo rm -r .git/



