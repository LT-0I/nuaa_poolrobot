#include <public.h>
#include <bringup.h>

using namespace std;

BringUp::BringUp()
{
    key_t key = ftok(PATH_NAME, PROJ_ID);
    int shmid = shmget(key, sizeof(ares_cmd), 0666 | IPC_CREAT);
    ares_cmd = (AresCmd *)shmat(shmid, 0, 0);

    ares_cmd->mapping_onff_cmd = MDU_EMPTY_MAP_CMD;
    module_state.onoff_states.mapping_onoff_sta = MDU_MAP_EMPTY_STA;

    ares_cmd->location_onff_cmd = MDU_START_LOCATION_CMD;
    module_state.onoff_states.location_onoff_sta = MDU_LOCATION_RUNNING_STA;
}

BringUp::~BringUp()
{
}

bool BringUp::checkApplication(std::string appName)
{
    int check_out;
    char cmd[200];

    // Prepare cmd
    std::string cmd_t = "ps -ef | grep " + appName + " | grep -v grep | wc -l";
    std::strcpy(cmd, cmd_t.c_str());

    FILE *popen_check = popen(cmd, "r");
    if (!popen_check)
    {
        std::perror("Bash shell check open error!");
        return false;
    }
    else
    {
        char tmp[1024];
        while (fgets(tmp, sizeof(tmp), popen_check) != NULL)
        {
            sscanf(tmp, "%d", &check_out);
        }
        pclose(popen_check);
    }

    return (check_out == 1) ? true : false;
}

bool BringUp::killApplication(std::string appName)
{
    char app_name[100];
    std::strcpy(app_name, appName.c_str());

    if (checkApplication(appName) == true)
    {
        char cmd[200];
        std::string cmd_t = "pkill -f " + appName;
        std::strcpy(cmd, cmd_t.c_str());

        FILE *popen_kill = popen(cmd, "r");
        if (!popen_kill)
        {
            std::perror("Bash shell kill open error!");
            return false;
        }
        else
            pclose(popen_kill);
    }
    else
        ROS_INFO("-> No thread named %s is running", app_name);

    return true;
}

void BringUp::runMappingApp()
{
    // if (checkApplication("mapping_node") == false)
    // {
        popen_cmdRun = popen("~/Sansy/src/ui_run_mapping.sh", "r");
        if (!popen_cmdRun)
        {
            std::perror("run mapping error");
        }
        else
        {
            ROS_INFO("-> Starting Mapping ...");
        }
    // }
    // else
    //     ROS_INFO("-> mapping thread is running");
}

void BringUp::stopMappingApp()
{
    if (killApplication("mapping_node"))
        ;
    ROS_INFO("-> Stop Mapping ...");
}

void BringUp::runLocationApp()
{
    // if (checkApplication("location_node") == false)
    // {
        popen_cmdRun = popen("~/Sansy/src/ui_run_location.sh", "r");
        if (!popen_cmdRun)
        {
            std::perror("run localization");
        }
    // }
    // else
    //     ROS_INFO("-> Localization thread is running");
}

void BringUp::stopLocationApp()
{
    if (killApplication("location_node"))
        ;
    ROS_INFO("-> Stop Location");
}

void BringUp::motionStateCallback()
{
}

void BringUp::mappingStateCallback()
{
}

void BringUp::sendAGVStates()
{
    // mars_state_pub_.publish(mars_state);
    // mars_error_pub_.publish(mars_error);
    // heart_beats_pub_.publish(heart_beats);
    // device_state_pub_.publish(device_state);
    // module_state_pub_.publish(module_state);
}

void BringUp::subNavigaMode(const mars_msgs::MarsStates naviga_mode_stamsg)
{
    mars_state.mars_ctlmode_sta = naviga_mode_stamsg.mars_ctlmode_sta;
}

void BringUp::MapingModule()
{
    // cout << "ares_cmd->mapping_onff_cmd = " << (int)ares_cmd->mapping_onff_cmd << endl;
    // cout << "mars_state.mars_ctlmode_sta = " << (int)mars_state.mars_ctlmode_sta << endl;

    // if (mars_state.mars_ctlmode_sta == MARS_MANUAL_MODE_STA)
    // {
    
    // Start Mapping APP
    if (ares_cmd->mapping_onff_cmd == MDU_DRAW_MAP_CMD && module_state.onoff_states.mapping_onoff_sta != MDU_MAP_DRAWING_STA)
    {
        cout << "开始建图" << endl;
        runMappingApp();
        module_state.onoff_states.mapping_onoff_sta = MDU_MAP_DRAWING_STA;
        module_state_pub_.publish(module_state);
    }

    // Stop Mapping APP and Save map
    if (ares_cmd->mapping_onff_cmd == MDU_STOP_MAP_CMD && module_state.onoff_states.mapping_onoff_sta != MDU_MAP_SAVING_STA)
    {
        cout << "停止建图" << endl;
        module_state.onoff_states.mapping_onoff_sta = MDU_MAP_SAVING_STA;
        module_state_pub_.publish(module_state);
    }
    // }

    // Start Location APP
    if (ares_cmd->location_onff_cmd == MDU_START_LOCATION_CMD && module_state.onoff_states.location_onoff_sta != MDU_LOCATION_RUNNING_STA)
    {
        cout << "开启定位" << endl;
        runLocationApp();
        module_state.onoff_states.location_onoff_sta = MDU_LOCATION_RUNNING_STA;
        ares_cmd->mapping_onff_cmd = MDU_EMPTY_MAP_CMD;
        module_state.onoff_states.mapping_onoff_sta = MDU_MAP_EMPTY_STA;
        module_state_pub_.publish(module_state);
    }

    // Stop Location APP
    if (ares_cmd->location_onff_cmd == MDU_STOP_LOCATION_CMD && module_state.onoff_states.location_onoff_sta != MDU_LOCATION_STOP_STA)
    {
        cout << "停止定位" << endl;
        module_state.onoff_states.location_onoff_sta = MDU_LOCATION_STOP_STA;
        module_state_pub_.publish(module_state);
    }
}

void BringUp::rosNode()
{
    mars_state_pub_ = nh_.advertise<mars_msgs::MarsStates>("/mars_state", 1, true);
    mars_error_pub_ = nh_.advertise<mars_msgs::MarsError>("/mars_error", 1, true);
    heart_beats_pub_ = nh_.advertise<mars_msgs::HeartBeats>("/heart_beats", 1, true);
    device_state_pub_ = nh_.advertise<mars_msgs::DeviceStates>("/device_state", 1, true);
    module_state_pub_ = nh_.advertise<mars_msgs::ModuleStates>("/module_state", 1, true);

    naviga_mode_sub_ = nh_.subscribe("naviga_mode", 100, &BringUp::subNavigaMode, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bring_up_node");
    ros::start();

    BringUp bring_up;
    bring_up.rosNode();

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        bring_up.MapingModule();
        // bring_up.sendAGVStates();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
}