[![IMAGE ALT TEXT](http://img.youtube.com/vi/zhzNXOa8b_M/0.jpg)](http://www.youtube.com/watch?v=zhzNXOa8b_M "ROS Integration of the Single Shot Detector for german traffic signs")

#Installation
- Um das Package installieren zu können sind Python in der Version 2.7.x notwendig (In der ROS Installation bereits enthalten).
- Das Package benötigt die Python-Bibliotheken
-- PIP (sudo apt install python-pip)
-- Numpy (sudo pip install numpy)
-- Pandas (sudo pip install pandas)
-- Path (sudo pip install Path)
-- Tensorflow (sudo pip install tensorflow)
#Ausführung
Um das Package ausführen zu können muss das Projekt zuvor in dem catkin Workspace gebaut werden. 
- Vor dem Bauen müssen die Dateien .msg/ .srv und *.py im Ordner des Projektes ros_tsr_s4b ausführbar gemacht werden.
$ cd ${PFAD_ZUM_CATKIN_WS}/src/ros_tsr_s4b
$ chmod a+x msg/*.msg srv/.*.srv src/*.py

Um das Projekt zu bauen, muss der Befehl

$ cd ${PFAD_ZUM_CATKIN_WS}
$ catkin_make

und danach

source ${PFAD_ZUM_CATKIN_WS}/devel/setup.bash 

ausgeführt werden

alle zur Verfügung stehenden Topics und Services des Projektes können mit 
$ rostopic list
beziehungsweise mit
$ rosservice list
ermittelt werden
