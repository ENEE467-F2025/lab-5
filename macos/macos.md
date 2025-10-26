# Instructions for running the Docker container on MacOS with X11 forwarding
## Step-By-Step Guide

1. Install XQuartz via brew

    `$ brew install --cask xquartz`
 
2. Logout and login of your Mac to activate XQuartz as default X11 server

3. Start XQuartz

    `$ open -a XQuartz`

4. Go to Security Settings and ensure that "Allow connections from network clients" is on

    ![alt XQuartz Security Stettings](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088/raw/d6eb9e8b0c20e51c46c5c9eb733b7f5e1144af4f/xquartz_preferences.png "XQuartz Security Settings")
    
5. Restart your Mac and start XQuartz again`

    `$ open -a XQuartz`

6. Check if XQuartz is setup and running correctly
    
    `$ ps aux | grep Xquartz`

7. Ensure that XQuartz is running similar to this: `/opt/X11/bin/Xquartz :0 -listen tcp`
    
    :0 means the display is running on display port 0.
    Important is that its not saying `–nolisten tcp` which would block any X11 forwarding to the X11 display.

8. Allow X11 forwarding via xhost

    `$ xhost +`
    
    This allows any client to connect. If you have security concerns you can append an IP address for a whitelist mechanism.
	
	Alternatively, if you want to limit X11 forwarding to local containers, you can limit clients to localhost only via
    
	`$ xhost +localhost`
	
	Be ware: You will always have to run `xhost +` after a restart of X11 as this is not a persistent setting.

9. Time to test X11 forwarding

    Run the docker container making sure to set the display variable
    ```
    $ docker compose -f lab-5-compose.yml run --rm -e DISPLAY=host.docker.internal:0 lab-5-docker
    ```