@echo off
cd src/algorithms
start cmd /c "miniros install || pause"

cd ../constants 
start cmd /c "miniros install || pause"

cd ../goal_manager
start cmd /c "miniros install || pause"

cd ../lidar
start cmd /c "miniros install || pause"

cd ../motion_controller
start cmd /c "miniros install || pause"

cd ../path_planner 
start cmd /c "miniros install || pause"

cd ../slam
start cmd /c "miniros install || pause"

cd ../..