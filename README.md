# inverse_kinematic

this package has two nodes first read from leapMotion and publish the wrist position and palm orientation to the second package which transform the position to angles and publish them to the robotic arm or model


steps :
      start the leapMotion:
       
          sudo leapd
    
      launch the IK:
      
          roslaunch inverse_kinematic demo.launch
          
