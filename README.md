# BbbNmea2k
simple indicator depth-speed-temp NEMA 2000 DST800
simple transmitter pressure, magneto(heading), angle(roll)

DST800 Triducer Airmar NEMA2000
Beaglebone Blue
	Can Interface from DST800
	Embedded I2C sensors
	Can Interface to host
LCD Output SPI

"BeagleBone Blue I2C sensors reader to CAN bus NMEA 2000 writer"
"DST800 NMEA 2000 CAN reader to BeagleBone Data out: console, SPI LCD, SSH... "

# Introduction 
...

# Getting Started
CAN: install package

apt-get install can-utils

git clone https://git.pengutronix.de/git/tools/libsocketcan
~$ cd libsocketcan
~/libsocketcan$ ./autogen.sh
~/libsocketcan$ ./configure
~/libsocketcan$ make
~/libsocketcan$ sudo make install

Add in file: /etc/rc.local
 echo "temppwd" | sudo -S ip link set can0 up type can bitrate 250000

# Build and Test
...

# Contribute
...

# Connect to wifi
sudo -s (become superuser/root)
connmanctl
connmanctl> tether wifi off (not really necessary on latest images)
connmanctl> enable wifi (not really necessary)
connmanctl> scan wifi
connmanctl> services (at this point you should see your network appear along with other stuff, in my case it was "AR Crystal wifi_f45eab2f1ee1_6372797774616c_managed_psk")
connmanctl> agent on
connmanctl> connect wifi_f45eab2f1ee1_6372797774616c_managed_psk
connmanctl> quit

# Wifi Ap
What is the name of the access point SSID and password default on BeagleBone Blue?
SSID: BeagleBone-XXXX where XXXX is based upon the board's assigned unique hardware address
Password: BeagleBone

First disable and stop the existing bone101 stuff, as per Bas Wijnen's answer:
systemctl disable bonescript.socket
systemctl disable bonescript.service
systemctl stop bonescript.socket
systemctl stop bonescript.service
Then edit the apache web server configuration. Start by editing the port listening configuration:

sudo vim /etc/apache2/ports.conf
and change this line:
Listen 8080
to this:
Listen 80
as port 80 is the default port for http traffic. Otherwise people would have to go to www.your-website.com:8080 which is just silly.
Then, as noted in the above file, you will also have to edit the sites enabled configuration:
sudo vim /etc/apache2/sites-enabled/000-default.conf
Edit the first line from this:
<VirtualHost *:8080>
to this:
<VirtualHost *:80>
Then either put your content in the directory noted in the DocumentRoot field (this is the directory to be used as the root of your website), which is by default /var/www/html, or change the DocumentRoot field to point to the directory you want to use. In my case, I left it as /var/www/html but then made that a symlink to a directory within a git repository where my website content is.
Or if you just want to do a quick test, symlink or copy the apache default test page into the DocumentRoot directory or change the field to point to the directory where the apache default test page is located (by default, /usr/share/apache2/default-site/index.html)
Then restart apache, et voilà:
sudo /etc/init.d/apache2 restart


