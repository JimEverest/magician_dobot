#include <magician_hardware/magician_hardware_interface.h>

typedef struct{
    controller_manager::ControllerManager *manager;
    magician_hardware::MagicianHWInterface *interface;
}ArgsForThread;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

void *update_loop(void *threadarg)
{
    ArgsForThread *arg=(ArgsForThread *)threadarg;
    controller_manager::ControllerManager *manager=arg->manager;
    magician_hardware::MagicianHWInterface *interface=arg->interface;
    ros::Duration d(0.01);
    struct timespec tick;
    clock_gettime(CLOCK_REALTIME, &tick);
    //time for checking overrun
    struct timespec before;
    double overrun_time;
    while(ros::ok())
    {
        ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
        interface->read(this_moment, d);
        manager->update(this_moment, d);
        interface->write(this_moment, d);
        timespecInc(tick, d.nsec);
        // check overrun
        clock_gettime(CLOCK_REALTIME, &before);
        overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
        if(overrun_time > 0.0)
        {
            tick.tv_sec=before.tv_sec;
            tick.tv_nsec=before.tv_nsec;
            std::cout<<"overrun_time: "<<overrun_time<<std::endl;
        }

        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
    }
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        ROS_ERROR("[USAGE]Application portName");
        return -1;
    }
    // Connect Dobot before start the service
    int result = ConnectDobot(argv[1], 115200, 0, 0);
    switch (result) {
        case DobotConnect_NoError:
        break;
        case DobotConnect_NotFound:
            ROS_ERROR("Dobot not found!");
            return -2;
        break;
        case DobotConnect_Occupied:
            ROS_ERROR("Invalid port name or Dobot is occupied by other application!");
            return -3;
        break;
        default:
        break;
    }
    ros::init(argc, argv, "magician_hardware_node", ros::init_options::AnonymousName);

    magician_hardware::MagicianHWInterface magician_hw_interface;
    controller_manager::ControllerManager ctlr_manager(&magician_hw_interface);
    ros::NodeHandle n1, n2;
    magician_hw_interface.init(n1, n2);

    pthread_t tid;
    ArgsForThread *thread_arg=new ArgsForThread();
    thread_arg->manager=&ctlr_manager;
    thread_arg->interface=&magician_hw_interface;

    pthread_create(&tid, NULL, update_loop, thread_arg);

    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
}
