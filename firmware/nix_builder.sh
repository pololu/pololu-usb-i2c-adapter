source $stdenv/setup

cp --no-preserve=mode -r $src firmware
cd firmware
bash ./build.sh
chmod a-x build_release/firmware.elf
arm-none-eabi-strip build_release/firmware.elf
mkdir -p $out
cp build_release/firmware.{elf,list,map} $out/
arm-none-eabi-size build_release/firmware.elf > $out/firmware.size
