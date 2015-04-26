TuioKinect tracks simple hand gestures using the Kinect controller and sends control data based on the <a href='http://www.tuio.org/'>TUIO</a> protocol. This allows the rapid creation of gesture enabled applications with any platform or environment that supports TUIO. Check out this extensive list of TUIO enabled software for further information: http://www.tuio.org/?software<p>


<a href='http://www.youtube.com/watch?feature=player_embedded&v=vZSEEnMP6pg' target='_blank'><img src='http://img.youtube.com/vi/vZSEEnMP6pg/0.jpg' width='425' height=344 /></a><br>
<br>
<br>
<b>Note:</b> This is a preliminary proof of concept implementation, which still needs several improvements to become fully usable. Nevertheless it should work out of the box with most TUIO client applications.<br>
<br>
<b>Configuration:</b> You can adjust the near/far threshold using the -/+ and </> keys.<br>
<br>
<b>Todo:</b>
<li>improve calibration by defining an active plane/region in 3D space rather than using two simple distance thresholds<br>
<li>color/depth calibration to enable more detailed segmentation of the hands<br>
<li>open/closed/pointing hand recognition to distinguish dedicated tracking modes.<br>
<li>create a windows project and binaries<br>
<li>create a osx 10.5 universal binary<br>
<br>
<b>Acknowledgments:</b>
This application has been created by <a href='http://modin.yuri.at'>Martin Kaltenbrunner</a> at the <a href='http://www.interface.ufg.ac.at'>Interface Culture Lab</a>. It is based on <a href='http://www.openframeworks.cc'>OpenFrameworks</a> and its <a href='http://github.com/ofTheo/ofxKinect'>ofxKinect</a> example, which is integrating <a href='https://github.com/OpenKinect/libfreenect'>libfreenect</a> by the <a href='http://openkinect.org/'>OpenKinect</a> project. Many thanks to all the developers involved in the process, without their work this application wouldn't have been that easy to implement.<p>

In order to compile this application you will need a working installation of OpenFrameworks and the ofxKinect extension. The provided binary has been compiled and tested under Mac Os 10.6 on an Intel machine only.<p>