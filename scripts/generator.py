import sys


''' This is the generator python script for generating the .launch file for n-agents.
    The script needs to be improved to accomomodate additions at a later point.
    
    TODO:
        Port to xml tree parser with some library like lxml for python.
        
'''
def main( n):
    print "<launch>"
    print """ <include file="$(find gazebo_ros)/launch/empty_world.launch">     
    <arg name="world_name" value="$(find swarm_simulator)/world/arena.world"/>      
  </include>  

 """
    for i in range(0,n):
    	newbot=""
        botname="swarmbot"
        botname+=str(i) 
        newbot+="""<group ns="""
        newbot+="\"" + str(botname) + "\""
        newbot+=""">\n"""
        newbot+="""<include file="$(find swarm_simulator)/launch/include/one_swarmRobot.launch">
"""
        botID=i
        newbot+="""<arg name="robot_name" value="""
        newbot+="\"" + str(botname) + "\""
        newbot+="""/>\n"""
        newbot+="""<arg name="init_pose" value="""
        x = -10+0.5*i
        y = -10+0.5*i
        newbot+="\"" + "-x " + str(x) + " -y " + str(y) + " -z 0" +"\"" + "/>\n"
        newbot+="</include>\n"
        newbot+="""</group>\n"""
    	print newbot
    print """ 
    <node pkg="swarm_simulator" type="swarm_simulator_node" name="swarm_simulator_node" respawn="false" /> """
    print "</launch>"

if __name__ == '__main__':
	main(int(sys.argv[1]))
