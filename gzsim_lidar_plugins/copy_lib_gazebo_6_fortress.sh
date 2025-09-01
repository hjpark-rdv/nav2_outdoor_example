cp libRobotecGPULidar.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
cp libRGLServerPluginInstance.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
cp libRGLServerPluginManager.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/
cp libRGLVisualize.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/gui/

sudo chmod 777 /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libRGL*
sudo chmod 777 /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/libRobotecGPULidar.so
sudo chmod 777 /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/gui/libRGLVisualize.so
