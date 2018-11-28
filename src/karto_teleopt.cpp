
 
#include <stdlib.h>
#include <linux/input.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
  
#include <thread>  

#include <ros/ros.h>  
#include <geometry_msgs/Twist.h>  

#include <iostream>
  
using namespace std;
  
class TeleoptNode  
{  
  
    public:  
        TeleoptNode():
        use_km_control_(false),
        km_update_rate_(200),
        km_thread_(new thread(&TeleoptNode::keyMouseLoop, this))
        {  
            pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  
              
            ros::NodeHandle n_private("~");  
            n_private.param("x_vel", x_vel_, 0.0);  
            n_private.param("y_vel", y_vel_, 0.0);  
            n_private.param("yaw_rate", yaw_rate_, 0.0);
        }  
          
        ~TeleoptNode() {
            if(km_thread_->joinable()){
                km_thread_->join();
            }  
            delete km_thread_;
         }  
        void keyMouseLoop();  
          
        void stopRobot()  
        {  
            cmdvel_.linear.x = 0.0;  
            cmdvel_.linear.y = 0.0; 
            cmdvel_.angular.z = 0.0;  
            pub_.publish(cmdvel_);  
        }  

        void ClearVel(void){
            x_vel_ = 0.0;
            y_vel_ = 0.0;
            yaw_rate_ = 0.0;
        }
        
    private:
        double x_vel_;  
        double y_vel_;  
        double yaw_rate_;   

        bool use_km_control_;
          
        geometry_msgs::Twist cmdvel_;  
        ros::NodeHandle n_;  
        ros::Publisher pub_;  

        unsigned int km_update_rate_;
        thread* km_thread_;
        int fd,fd_kb,fd_mouse;
};  


void TeleoptNode::keyMouseLoop(void)  
{  
    int retval;
    fd_set readfds;
    struct timeval tv;

    char event_mouse_move[6];
    fd = open( "/dev/input/mice", O_RDONLY );
    // 判断是否打开成功
    if(fd<0) {
        printf("Failed to open \"/dev/input/mice\".\n");
        exit(1);
    } else {
        printf("open \"/dev/input/mice\" successfuly.\n");
    }

    struct input_event event_kb;
    fd_kb = open("/dev/input/event4", O_RDONLY); //键盘输入
    if(fd_kb <= 0)
    {
            printf("open device error\n");
            return;
    }

    struct input_event event_mouse_key;
    fd_mouse = open("/dev/input/event4", O_RDONLY); //鼠标输入
    if(fd_mouse <= 0)
    {
            printf("open device error\n");
            return;
    }

    while(ros::ok())
    {  
        tv.tv_sec = 5;
        tv.tv_usec = 0;

        FD_ZERO( &readfds );
        FD_SET( fd, &readfds );
        FD_SET( fd_kb, &readfds );
        FD_SET( fd_mouse, &readfds );

        int max=fd>fd_kb?(fd>fd_mouse?fd:fd_mouse):(fd_kb>fd_mouse?fd_kb:fd_mouse);

        retval = select( max+1, &readfds, NULL, NULL, &tv );//有文件描述符的状态发生变化（变为可读）或者超时和错误就返回，否则就阻塞
        if(retval==0) {
            printf( "Time out!\n" );
        }else if(retval<0){
            printf( "error!\n" );
        }else{						//判断是哪个文件描述符的状态发生了变化，进行处理即可

            //鼠标位置
            if(FD_ISSET(fd,&readfds))
            {
                // 读取鼠标设备中的数据
                if(read(fd, event_mouse_move, 6) <= 0) {
                    continue;
                }
                // 打印出从鼠标设备中读取到的数据
                // printf("Button type = %d, X = %d, Y = %d, Z = %d\n", (event_mouse_move[0] & 0x07), event_mouse_move[1], 
                //                                                       event_mouse_move[2], event_mouse_move[3]);
                yaw_rate_ = (event_mouse_move[1] >= 0) ? ((event_mouse_move[1] == 0) ? (0) : (-1)) : (1);
            }

            //键盘点击
            if(FD_ISSET(fd_kb,&readfds))
            {
                if(read(fd_kb, &event_kb, sizeof(event_kb)) == sizeof(event_kb))
                {
                    if (event_kb.type == EV_KEY)
                    {
                        //if (event_kb.value == 0 || event_kb.value == 1)//1表示按下，0表示释放，会检测到两次
                        if (event_kb.value == 1)//键按下
                        {
                                // printf("key %d %s\n", event_kb.code, (event_kb.value) ? "Pressed" : "Released");
 
                                if(event_kb.code == KEY_W){
                                    x_vel_ = 1;
                                }
                                if(event_kb.code == KEY_S){
                                    x_vel_ = -1;
                                }
                                if(event_kb.code == KEY_A){
                                    y_vel_ = 1;
                                }
                                if(event_kb.code == KEY_D){
                                    y_vel_ = -1;
                                }

                                if(event_kb.code == KEY_ESC){
                                    use_km_control_ = false;
                                }
                                if(event_kb.code == KEY_ENTER){
                                    use_km_control_ = true;
                                    cout << "W:front  S:back  A:left  D:right \r\n" << endl;
                                }
                        }else if(event_kb.value == 0){
                            if(event_kb.code == KEY_W){
                                    x_vel_ = 0;
                                }
                                if(event_kb.code == KEY_S){
                                    x_vel_ = 0;
                                }
                                if(event_kb.code == KEY_A){
                                    y_vel_ = 0;
                                }
                                if(event_kb.code == KEY_D){
                                    y_vel_ = 0;
                                }
                        }

                    }

                }
            }

            //鼠标点击
            if(FD_ISSET(fd_mouse,&readfds))
            {
                if(read(fd_mouse, &event_mouse_key, sizeof(event_mouse_key)) == sizeof(event_mouse_key))
                {
                    if(event_mouse_key.type = EV_REL)
                    {
                        if(event_mouse_key.code == REL_WHEEL)
                            printf("REL_WHEEL %d\n", event_mouse_key.value);//-1表示下滑，1表示上滑
                    }

                    if(event_mouse_key.code == BTN_LEFT && event_mouse_key.value==1)//左键按下，1表示按下，0表示释放。不然会检测到两次
                        printf("left down: %d\n", event_mouse_key.code);

                    if(event_mouse_key.code == BTN_RIGHT && event_mouse_key.value==1)
                        printf("right down: %d\n", event_mouse_key.code);

                    if(event_mouse_key.code == BTN_MIDDLE && event_mouse_key.value==1)
                        printf("middle down: %d\n", event_mouse_key.code);

                }
            }
        }

        if(use_km_control_){
            cmdvel_.linear.x = x_vel_;  
            cmdvel_.linear.y = y_vel_; 
            cmdvel_.angular.z = 0.7 * yaw_rate_;  
            pub_.publish(cmdvel_);  
            yaw_rate_ = 0.0;
        }
        
        // cout << "pub cmd vel : " << "  x_vel_ " << x_vel_ 
        //                          << "  y_vel_ " << y_vel_ 
        //                          << "  yaw_rate_ " << yaw_rate_ 
        //                          << endl;
        
    }  



    close(fd);
    close(fd_kb);
    close(fd_mouse);

    return;
}

int main(int argc, char** argv)  
{  
    ros::init(argc,argv,"teleopt_nav", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);  
      
    
    TeleoptNode teleopt_node;  
    ros::spin();
   

      
    return(0);  
}  

