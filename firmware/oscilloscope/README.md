This little helper is used to display the perimeter signal from Mowgli. Compile it on the rasperrypi. The compile time requirements are installed on Ubuntu using

        sudo apt install gcc automake autoconf libgtk-3-dev

Then execute

        automake --add-missing
        autoreconf
        ./configure
        make
        sudo make install

to compile and install. 

The command "oscilloscope" will open an X-window and display the debug output from the perimeter signal. To enable the debug output for the left coil execute

        rosservice call mower_service/permeter_listen 128

Use 129 or 130 for the center or right coil.