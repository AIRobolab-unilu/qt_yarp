Convering videos to a smaller size and different frame rates 
-------------------------------------------------------------
The LSV7 resolution on odroid is 1280x720! 

- to X264 codec
`$ ffmpeg -y -i happy.avi -r 4 -s 640x360 -vcodec libx264 -b 512K -strict -2 happy_small.avi` 

- to raw frames
`$ ffmpeg -y -i happy.avi -r 5 -an -s 640x360 -vcodec rawvideo  happy_raw.avi`

- convert to gif
`$ ffmpeg -i happy.avi -r 4 -s 128x72 -f image2pipe -vcodec ppm - | convert -delay 5 - gif:- | convert -layers Optimize - output.gif`

