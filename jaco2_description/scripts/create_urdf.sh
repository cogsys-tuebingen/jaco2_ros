#!/bin/bash
for f in ../robots/*.urdf.xacro; 
	do	
	output=${f%.xacro};
	echo $output
	rosrun xacro xacro --inorder -o $output $f
	done;

