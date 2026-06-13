
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

import sys, select, termios, tty


msg = """
Reading from the keyboard  and Publishing to Twist!
Keyboard control mobot
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c
q/e : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
z/c : increase/decrease only angular speed by 10%
a/d : left/right turn by 10%
CTRL-C to quit
"""

moveBindings = {
        'w':( 0.01, 0, 0, 0, 0, 0),
        'x':(-0.01, 0, 0, 0, 0, 0),
        'a':( 0, 0.3, 0, 0, 0, 0),
        'd':( 0,-0.3, 0, 0, 0, 0),
        'q':( 0, 0, 0.01, 0, 0, 0),
        'e':( 0, 0,-0.01, 0, 0, 0),
        'z':( 0, 0, 0, 0.1, 0, 0),
        'c':( 0, 0, 0,-0.1, 0, 0),        
        's':( 0, 0, 0, 0,   1, 0.1),
            }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.13)  
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('mogo_teleop_keyboard')
    vx = 0
    vy = 0
    x = 0
    th = 0
    cs = 0
    ca = 0
    st = 0
    count = 0
    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                cs = moveBindings[key][2]
                ca = moveBindings[key][3]
                st = moveBindings[key][4]
                count = 0
                if st == 1:
                   x = 0
                   cs = 0
                   vx = 0
                   vy = 0
                   th = 0 
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                    cs = 0
                    ca = 0
                    st = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            vx = vx + x
            if vx>0.3:
                vx = 0.3
            if vx<-0.3:
                vx = -0.3
            vy = vy + cs
            twist.linear.x = vx
            # twist.linear.x = cs
            twist.linear.y = vy
            twist.linear.z = 0

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th
            pub.publish(twist)
    except Exception as e:
        print(e)

    finally:            
        twist = Twist()
        twist.linear.x = 0 
        twist.linear.y = 0 
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        pub.publish(twist)
    
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)



