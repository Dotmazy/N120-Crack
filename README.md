# N120-Crack
In this repository there is all the source code i've made (bootloader) and used to crack the numwork n120.
If someone wanna help me cracking the n120 (if you're french it's even better because i'm french) send me a request on discord "dotmazy".

# DO NOT FOLLOW THIS TUTORIAL, IT'S WORK IN PROGRESS

# Here is the step to crack the n120 (WIP)

## Needed

Here is all the things you'll need:
- a st link (not needed if your calculator isn't locked)
- a numwork N120 (look at the back of your calculator)
- 1/3 cables
- a screw driver

## Open the numwork

First if you have any scripts or values don't forget to save it in your computer.
Next open the numwork, be carefull and unplug the battery.

## Go into boot0 mode
Download the bootloader in **Release** Section.
## (WIP, please wait)
Connect the numwork to the pc.
Connect the pin 3.3v to boot0 (seen in the image) then click reset on the calculator (put the black part to click on it). If the light turn red then it's ok, else if the led turn green that mean that your calculator is locked, skip this step:

## Flash the custom bootloader (unlocked)
Go to [Webdfu](https://ti-planet.github.io/webdfu_numworks/n0110/), select the `internal.bin` file and click `Flash Internal`.

## Your calculator is locked
If there is an error don't panic it's just that your numwork is locked (like the n110 in some cases), to unlock it you will need to buy an st link or a raspberry pi that tou can use as an st link. I'll show you how to do it directly with an st link (here is one that you can take [from aliexpress](https://fr.aliexpress.com/item/1005005273159580.html?src=google&pdp_npi=4%40dis!EUR!2.31!2.31!!!!!%40!12000032440955298!ppc!!!&src=google&albch=shopping&acnt=248-630-5778&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&&albagn=888888&&ds_e_adid=&ds_e_matchtype=&ds_e_device=m&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=fr1005005273159580&ds_e_product_merchant_id=559096839&ds_e_product_country=FR&ds_e_product_language=fr&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=20180143335&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gclid=CjwKCAiAtNK8BhBBEiwA8wVt98MMo_ckNx27aYHsyJgSvaeyl5_o8L3p0y5lY9wqSC2Fe72sFBr3OBoCVFcQAvD_BwE)).
On the st link you can see the pins just connect three cables: one on swclk, one on swdio and one on gnd. Connect gnd to the gnd of the calculator (just put it on top of the usb c port), swdclk to swclk on the bottom left pins and swdio to swdio (bottom left pins are on the image bellow)

When you have done that connect the numwork to your pc, on your pc install [STM32 Cube Programmer](https://www.st.com/en/development-tools/stm32cubeprog.html) and launch it, select st link on the right if it's not already selected then click on connect (verify that the pins between the numwork and the st link are connected well) then go in OB on the right, then extends Read Out Protection there should be CC, set it to AA and also go down to Write Protection and turn everything on (it will turn of the write protection). Then you can either go back to Webdfu or use STM32CubeProgrammer to flash the bootloader.

##Done
If you're here the bootloader is flashed and you're numwork n120 has been cracked :), yes it's finaly possible.

Then just go and install a custom firmware like omega or upsilon (note: idk if it will work).
