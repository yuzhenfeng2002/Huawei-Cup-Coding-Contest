#include <iostream>
#include "./Cmap.h"
#include "./Handle.h"
#include "./Robot.h"
using namespace std;


bool readUntilOK() {
    char line[1024];
    while (fgets(line, sizeof line, stdin)) {
        if (line[0] == 'O' && line[1] == 'K') {
            return true;
        }
        //do something
        cout<<line<<endl;
    }
    return false;
}



int main() {
    readUntilOK();
    puts("OK");
    fflush(stdout);
    Map m;
    int frame;
    while (scanf("%d", &frame) != EOF) {
        int money;
        cin>>money;
        m.update_map(frame, money);
        cin.get();
        int handle_num;
        cin>>handle_num;
        cin.get();
        if(frame == 1){
            m.init_handles(handle_num);
            m.init_robots();
        }
        for(int i=0; i<handle_num; i++){
            int handle_type;
            float x;
            float y;
            int left_time;
            int material;
            int object;
            cin>>handle_type>>x>>y>>left_time>>material>>object;
            m.update_handle(i,handle_type, x, y, left_time, material, object);
            cin.get();
        }

        for(int i=0; i<ROBO_NUM; i++){
            int handle_id;
            int object_type;
            double time_coeff;
            double crash_coeff;
            double rotate_speed;
            double speed_x;
            double speed_y;
            double direction;
            double x;
            double y;
            cin>>handle_id>>object_type>>time_coeff
                >>crash_coeff>>rotate_speed>>speed_x>>speed_y>>direction>>x>>y;
            m.update_robot(i, handle_id, object_type, time_coeff, crash_coeff, 
                rotate_speed, speed_x, speed_y, direction, x, y);
            cin.get();
        }

        if(frame == 1){
            m.update_handle_type_dict_first();
        }


        // printf("%d\n", frameID);
        // int lineSpeed = 3;
        // double angleSpeed = 1.5;
        // for(int robotId = 0; robotId < 4; robotId++){

        //     printf("forward %d %d\n", robotId, lineSpeed);
        //     printf("rotate %d %f\n", robotId, angleSpeed);
        // }
        readUntilOK();
        printf("%d\n", frame);
        
        m.output_strategy();
        fflush(stdout);
    }
    return 0;
}
